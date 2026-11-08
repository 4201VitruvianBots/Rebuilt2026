package frc.team4201.lib.logging;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.net.HttpURLConnection;
import java.net.URI;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.TouchpadFinger;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.hal.DriverStationJNI;
import org.wpilib.hardware.hal.can.CANStatus;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.networktables.ConnectionInfo;
import org.wpilib.networktables.GenericSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.util.AlertDataJNI;

/**
 * Mirrors the robot-side metrics that the Driver Station records in its own log (battery, brownout,
 * CAN health, DS/match state, controller load) into the robot's telemetry, so they land in the same
 * NetworkTables / .wpilog stream as the rest of your Epilogue data. Also logs the extra metadata
 * AdvantageKit records: epoch time, PDH power/energy, NT clients, GC, alerts, radio status, the
 * onboard IMU and joysticks, plus the Systemcore serial number, robot name (from the
 * web dashboard comments) and WiFi SSID.
 *
 * <p>Layout under {@code SystemLogger/}:
 *
 * <pre>
 * DSMetrics/     what the DS log records: battery, brownout, CAN utilization per bus, CPU use,
 *                comms disables, enabled/e-stop/mode/OpMode, DS and FMS attached
 * Power/         input voltage, 3.3V rail, PDH/PDP
 * CAN/&lt;bus&gt;/     error counts, bitrate, CAN FD, availability, frame rate
 * Match/         match time, alliance, location, event, match type/number, replay, game data
 * Controller/    serial number, robot name, epoch time, sys active, RSL, CPU temp, memory, disk
 * JVM/           heap, GC time and count per loop
 * Network/       NTClients/&lt;ip&gt;, Radio, WiFi
 * IMU/           onboard IMU
 * Joysticks/     Joystick0-5
 * Alerts/        active alerts per group
 * </pre>
 *
 * <p>Targets WPILib 2027.0.0-alpha-8 on Systemcore. Data is written through Epilogue's configured
 * {@link TelemetryTable}, so whatever backend Epilogue uses (NetworkTables by default) gets it too.
 *
 * <p>Not available from robot code: DS trip time, lost packets, and the DS laptop's own CPU. Those are
 * measured on the Driver Station computer and only exist in the DS log.
 *
 * <p>CAN bitrate, CAN FD and frame rate are not in WPILib's public API. They are read from
 * Systemcore's own {@code /diagnostics/canbusinfo} and {@code /diagnostics/canbusfps} NT topics (the
 * same source AdvantageKit uses). Those topics are undocumented and may change between images; if
 * they disappear, those fields read as 0/false.
 *
 * <p>Usage, in {@code Robot.java}:
 *
 * <pre>{@code
 * public Robot() {
 *   Epilogue.configure(config -> { ... });     // optional; do this first
 *   SystemLogger.setPowerDistribution(pdh); // optional, reuse your existing PDH/PDP object
 *   SystemLogger.setCANBuses(CANPort.CAN_S0, CANPort.CAN_S1); // optional, defaults to S0-S4
 *   SystemLogger.setIMUOrientation(OnboardIMU.MountOrientation.FLAT); // optional
 * }
 *
 * @Override
 * public void robotPeriodic() {
 *   CommandScheduler.getInstance().run();
 *   SystemLogger.update();
 * }
 * }</pre>
 */
public final class SystemLogger {
    /** Telemetry table name, created under Epilogue's configured table (default NT root). */
    private static final String TABLE_NAME = "SystemLogger";

    /** Number of Systemcore CAN buses (S0-S4) covered by the diagnostics topics. */
    private static final int SYSTEMCORE_CAN_BUSES = 5;

    /** Size of one packed CanBusInfoEntry: 4 doubles + 3 bools. */
    private static final int CAN_INFO_ENTRY_SIZE = 35;

    private static final double RADIO_PERIOD_SECS = 5.0;
    private static final int RADIO_TIMEOUT_MS = 500;

    // ---- configuration ----
    private static CANPort[] s_canBuses = {
            CANPort.CAN_S0, CANPort.CAN_S1, CANPort.CAN_S2, CANPort.CAN_S3, CANPort.CAN_S4
    };
    private static PowerDistribution s_powerDistribution;
    private static long s_systemPeriodNanos = 1_000_000_000L; // 1 Hz for /proc, NT clients, radio
    private static OnboardIMU.MountOrientation s_imuOrientation = OnboardIMU.MountOrientation.FLAT;
    private static boolean s_logJoysticks = true;
    private static boolean s_logRadio = true;

    // ---- tables (resolved lazily so Epilogue.configure() in the Robot constructor is honored) ----
    private static TelemetryTable s_root;
    private static TelemetryTable s_dsMetrics;
    private static TelemetryTable s_power;
    private static TelemetryTable[] s_canTables;
    private static TelemetryTable s_match;
    private static TelemetryTable s_controller;
    private static TelemetryTable s_jvm;
    private static TelemetryTable s_alerts;
    private static TelemetryTable s_ntClients;
    private static TelemetryTable s_radio;
    private static TelemetryTable s_wifi;
    private static TelemetryTable s_imu;
    private static TelemetryTable[] s_joystickTables;

    // ---- state ----
    private static long s_lastSystemUpdateNanos = Long.MIN_VALUE;
    private static long s_prevCpuTotal = -1;
    private static long s_prevCpuIdle = -1;
    private static boolean s_procAvailable = true;

    private static GenericSubscriber s_canInfoSub;
    private static GenericSubscriber s_canFpsSub;

    private static List<GarbageCollectorMXBean> s_gcBeans;
    private static long s_prevGcTimeMs = -1;
    private static long s_prevGcCount = -1;

    private static final Set<String> s_knownAlertGroups = new LinkedHashSet<>();
    private static final Set<String> s_knownClientIPs = new LinkedHashSet<>();

    private static OnboardIMU s_onboardIMU;

    private static Thread s_backgroundThread;
    private static volatile boolean s_radioConnected;
    private static volatile String s_radioStatusJson = "";
    private static volatile String s_wifiSSID = "";
    private static volatile String s_wifiMode = "";

    private static String s_serialNumber = "";
    private static String s_robotName = "";

    private SystemLogger() {}

    // ====================================================================================
    // Configuration
    // ====================================================================================

    /**
     * Sets which Systemcore/Motioncore CAN buses to report. Defaults to CAN_S0 through CAN_S4. Call
     * before the first {@link #update()}. Bitrate/FD/frame rate are only available for S0-S4.
     *
     * @param buses the buses to report
     */
    public static void setCANBuses(CANPort... buses) {
        s_canBuses = buses.clone();
    }

    /**
     * Reports per-channel currents, total current, voltage, temperature, total power and total energy
     * from a PDH/PDP. Pass the same instance the rest of your code uses rather than constructing a
     * second one.
     *
     * @param powerDistribution the PDH/PDP, or null to stop reporting it
     */
    public static void setPowerDistribution(PowerDistribution powerDistribution) {
        s_powerDistribution = powerDistribution;
    }

    /**
     * Sets how often the slower stats (CPU, memory, disk, NT clients, radio) are written. Defaults to 1
     * second.
     *
     * @param seconds sample period in seconds
     */
    public static void setSystemStatsPeriod(double seconds) {
        s_systemPeriodNanos = (long) (seconds * 1e9);
    }

    /**
     * Sets the Systemcore mount orientation used for the logged IMU yaw. Defaults to FLAT. Call before
     * the first {@link #update()}. This logger uses its own IMU handle, so it never resets the yaw your
     * drivetrain uses.
     *
     * @param orientation how Systemcore is mounted
     */
    public static void setIMUOrientation(OnboardIMU.MountOrientation orientation) {
        s_imuOrientation = orientation;
    }

    /**
     * Enables or disables joystick logging (on by default). Joysticks are the highest-volume group.
     *
     * @param enabled whether to log joysticks
     */
    public static void setLogJoysticks(boolean enabled) {
        s_logJoysticks = enabled;
    }

    /**
     * Enables or disables radio status polling (on by default). Polling runs on a background thread
     * every 5 seconds and only on a real robot.
     *
     * @param enabled whether to poll the radio
     */
    public static void setLogRadio(boolean enabled) {
        s_logRadio = enabled;
    }

    // ====================================================================================
    // Update
    // ====================================================================================

    /** Publishes all metrics. Call once per loop from {@code robotPeriodic()}. */
    public static void update() {
        if (s_root == null) {
            init();
        }

        // Every loop
        logPower();
//        logCAN();
        logDriverStation();
        logGC();
        logAlerts();
        logIMU();
        if (s_logJoysticks) {
            logJoysticks();
        }

        // Throttled
        long now = System.nanoTime();
        if (s_lastSystemUpdateNanos == Long.MIN_VALUE
                || now - s_lastSystemUpdateNanos >= s_systemPeriodNanos) {
            s_lastSystemUpdateNanos = now;
            logSystem();
            logNTClients();
            if (s_logRadio) {
                logRadio();
            }
        }
    }

    private static void init() {
        s_root = Epilogue.getConfig().table.getTable(TABLE_NAME);
        s_dsMetrics = s_root.getTable("DSMetrics");
        s_power = s_root.getTable("Power");
        s_match = s_root.getTable("Match");
        s_controller = s_root.getTable("Controller");
        s_jvm = s_root.getTable("JVM");
        s_alerts = s_root.getTable("Alerts");
        s_imu = s_root.getTable("IMU");
        TelemetryTable network = s_root.getTable("Network");
        s_ntClients = network.getTable("NTClients");
        s_radio = network.getTable("Radio");
        s_wifi = network.getTable("WiFi");

        TelemetryTable can = s_root.getTable("CAN");
        s_canTables = new TelemetryTable[s_canBuses.length];
        for (int i = 0; i < s_canBuses.length; i++) {
            s_canTables[i] = can.getTable(s_canBuses[i].name());
        }

        TelemetryTable joysticks = s_root.getTable("Joysticks");
        s_joystickTables = new TelemetryTable[DriverStationJNI.MAX_JOYSTICKS];
        for (int i = 0; i < s_joystickTables.length; i++) {
            s_joystickTables[i] = joysticks.getTable("Joystick" + i);
        }

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        s_canInfoSub = nt.getTopic("/diagnostics/canbusinfo").genericSubscribe();
        s_canFpsSub = nt.getTopic("/diagnostics/canbusfps").genericSubscribe();

        s_gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
        s_onboardIMU = new OnboardIMU(s_imuOrientation);

        s_serialNumber = RobotController.getSerialNumber();
        // The comments field from the Systemcore web dashboard, used as the robot's name. WPILib caches
        // it after the first read, so a change there shows up after the robot code restarts.
        String comments = RobotController.getComments();
        s_robotName = comments == null ? "" : comments.trim();

        // Radio and WiFi lookups are slow (HTTP / process exec), so they run off the robot thread.
        if (RobotBase.isReal()) {
            startBackgroundThread();
        }
    }

    // ====================================================================================
    // Power
    // ====================================================================================

    private static void logPower() {
        s_dsMetrics.log("BatteryVoltage", RobotController.getBatteryVoltage());
        s_dsMetrics.log("BrownedOut", RobotController.isBrownedOut());

        s_power.log("InputVoltage", RobotController.getInputVoltage());

//        s_power.log("Rail3V3/Voltage", RobotController.getVoltage3V3());
//        s_power.log("Rail3V3/Current", RobotController.getCurrent3V3());
//        s_power.log("Rail3V3/Enabled", RobotController.getEnabled3V3());
//        s_power.log("Rail3V3/FaultCount", RobotController.getFaultCount3V3());

        if (s_powerDistribution != null) {
            // PowerDistribution is TelemetryLoggable: logs Current[], Voltage, TotalCurrent,
            // SwitchableChannel under PowerDistribution/.
            s_power.log("PowerDistribution", s_powerDistribution);
            s_power.log("PowerDistributionTemp", s_powerDistribution.getTemperature());
            // Note: WPILib reports these as 0 on a REV PDH.
            s_power.log("PowerDistributionTotalPower", s_powerDistribution.getTotalPower());
            s_power.log("PowerDistributionTotalEnergy", s_powerDistribution.getTotalEnergy());
        }
    }

    // ====================================================================================
    // CAN
    // ====================================================================================

    private static void logCAN() {
        byte[] infoRaw = s_canInfoSub.getRaw(new byte[0]);
        double[] fps = s_canFpsSub.getDoubleArray(new double[0]);
        ByteBuffer info = ByteBuffer.wrap(infoRaw).order(ByteOrder.LITTLE_ENDIAN);
        int infoCount = infoRaw.length / CAN_INFO_ENTRY_SIZE;

        for (int i = 0; i < s_canBuses.length; i++) {
            CANPort bus = s_canBuses[i];
            CANStatus status = RobotController.getCANStatus(bus);
            TelemetryTable t = s_canTables[i];
            s_dsMetrics.log("CANUtilization/" + bus.name(), status.percentBusUtilization);
            t.log("BusOffCount", status.busOffCount);
            t.log("TxFullCount", status.txFullCount);
            t.log("ReceiveErrorCount", status.receiveErrorCount);
            t.log("TransmitErrorCount", status.transmitErrorCount);

            // Details from Systemcore's diagnostics topics, indexed by Systemcore bus number (S0-S4).
            int idx = bus.value;
            if (idx >= SYSTEMCORE_CAN_BUSES) {
                continue; // Motioncore buses aren't in the diagnostics topics
            }
            if (idx < infoCount) {
                int o = idx * CAN_INFO_ENTRY_SIZE;
                double nominalMbps = info.getDouble(o);
                double dataMbps = info.getDouble(o + 8);
                boolean fd = info.get(o + 32) != 0;
                boolean available = info.get(o + 33) != 0;
                boolean up = info.get(o + 34) != 0;
                t.log("MaxBandwidthMbps", fd ? dataMbps : nominalMbps);
                t.log("NominalBitrateMbps", nominalMbps);
                t.log("DataBitrateMbps", dataMbps);
                t.log("FD", fd);
                t.log("Available", available);
                t.log("InterfaceUp", up);
            }
            t.log("FrameRate", idx < fps.length ? fps[idx] : 0.0);
        }
    }

    // ====================================================================================
    // Driver Station state, match info, and per-loop controller state
    // ====================================================================================

    private static void logDriverStation() {
        s_dsMetrics.log("Enabled", RobotState.isEnabled());
        s_dsMetrics.log("EStopped", RobotState.isEStopped());
        s_dsMetrics.log("Mode", RobotState.getRobotMode().name());
        s_dsMetrics.log("OpMode", RobotState.getOpMode());
        s_dsMetrics.log("DSAttached", RobotState.isDSAttached());
        s_dsMetrics.log("FMSAttached", RobotState.isFMSAttached());
//        s_dsMetrics.log("CommsDisableCount", RobotController.getCommsDisableCount());

        Instant now = Instant.now();
        s_controller.log("EpochTimeMicros", now.getEpochSecond() * 1_000_000L + now.getNano() / 1_000);
        s_controller.log("EpochTimeValid", RobotController.isSystemTimeValid());
//        s_controller.log("SysActive", RobotController.isSysActive());
//        s_controller.log("RSLState", RobotController.getRSLState());

        s_match.log("MatchTime", MatchState.getMatchTime());
        s_match.log("Alliance", MatchState.getAlliance().map(Enum::name).orElse("UNKNOWN"));
        s_match.log("Location", MatchState.getLocation().orElse(0));
        s_match.log("EventName", MatchState.getEventName());
        s_match.log("MatchType", MatchState.getMatchType().name());
        s_match.log("MatchNumber", MatchState.getMatchNumber());
        s_match.log("ReplayNumber", MatchState.getReplayNumber());
        s_match.log("GameData", MatchState.getGameData().orElse(""));
    }

    // ====================================================================================
    // JVM garbage collection (per loop)
    // ====================================================================================

    private static void logGC() {
        long timeMs = 0;
        long count = 0;
        for (GarbageCollectorMXBean bean : s_gcBeans) {
            long t = bean.getCollectionTime();
            long c = bean.getCollectionCount();
            if (t > 0) {
                timeMs += t;
            }
            if (c > 0) {
                count += c;
            }
        }
        if (s_prevGcTimeMs >= 0) {
            s_jvm.log("GCTimeMS", (double) (timeMs - s_prevGcTimeMs));
            s_jvm.log("GCCount", count - s_prevGcCount);
        }
        s_prevGcTimeMs = timeMs;
        s_prevGcCount = count;
    }

    // ====================================================================================
    // Alerts
    // ====================================================================================

    private static void logAlerts() {
        AlertDataJNI.AlertInfo[] all = AlertDataJNI.getAlerts();
        Map<String, List<AlertDataJNI.AlertInfo>> activeByGroup = new HashMap<>();
        for (AlertDataJNI.AlertInfo a : all) {
            s_knownAlertGroups.add(a.group);
            if (a.activeStartTime != 0) {
                activeByGroup.computeIfAbsent(a.group, g -> new ArrayList<>()).add(a);
            }
        }

        // Groups with nothing active are still written so cleared alerts show as cleared.
        for (String group : s_knownAlertGroups) {
            List<AlertDataJNI.AlertInfo> active =
                    activeByGroup.computeIfAbsent(group, g -> new ArrayList<>());
            active.sort(
                    (x, y) -> {
                        int byTime = Long.compare(y.activeStartTime, x.activeStartTime); // newest first
                        return byTime != 0 ? byTime : x.text.compareTo(y.text);
                    });

            List<String> errors = new ArrayList<>();
            List<String> warnings = new ArrayList<>();
            List<String> infos = new ArrayList<>();
            for (AlertDataJNI.AlertInfo a : active) {
                switch (a.level) {
                    case AlertDataJNI.LEVEL_HIGH -> errors.add(a.text);
                    case AlertDataJNI.LEVEL_MEDIUM -> warnings.add(a.text);
                    default -> infos.add(a.text);
                }
            }

            TelemetryTable t = s_alerts.getTable(group.isEmpty() ? "Alerts" : group);
            t.setType("Alerts"); // lets AdvantageScope/Elastic render it as an alerts widget
            t.log("errors", errors.toArray(new String[0]));
            t.log("warnings", warnings.toArray(new String[0]));
            t.log("infos", infos.toArray(new String[0]));
        }
    }

    // ====================================================================================
    // Onboard IMU
    // ====================================================================================

    private static void logIMU() {
        OnboardIMU imu = s_onboardIMU;
        s_imu.log("AccelX", imu.getAccelX()); // m/s^2
        s_imu.log("AccelY", imu.getAccelY());
        s_imu.log("AccelZ", imu.getAccelZ());
        s_imu.log("GyroRateX", imu.getGyroRateX()); // rad/s
        s_imu.log("GyroRateY", imu.getGyroRateY());
        s_imu.log("GyroRateZ", imu.getGyroRateZ());
        s_imu.log("AngleX", imu.getAngleX()); // rad
        s_imu.log("AngleY", imu.getAngleY());
        s_imu.log("AngleZ", imu.getAngleZ());
        s_imu.log("YawRadians", imu.getYawRadians());
        s_imu.log("Rotation3d", imu.getRotation3d());
    }

    // ====================================================================================
    // Joysticks
    // ====================================================================================

    private static void logJoysticks() {
        for (int port = 0; port < s_joystickTables.length; port++) {
            TelemetryTable t = s_joystickTables[port];
            boolean connected = DriverStationBackend.isJoystickConnected(port);
            t.log("Connected", connected);
            if (!connected) {
                continue;
            }
            t.log("Name", DriverStationBackend.getJoystickName(port).trim());
            t.log("Type", DriverStationBackend.getJoystickGamepadType(port));
            t.log("IsGamepad", DriverStationBackend.getJoystickIsGamepad(port));

            // Buttons: 0-based bitmasks
            t.log("ButtonsAvailable", DriverStationBackend.getStickButtonsAvailable(port));
            t.log("ButtonValues", DriverStationBackend.getStickButtons(port));

            // Axes: only read available indices to avoid "unavailable axis" warnings
            int axesAvail = DriverStationBackend.getStickAxesAvailable(port);
            double[] axes = new double[32 - Integer.numberOfLeadingZeros(axesAvail)];
            for (int a = 0; a < axes.length; a++) {
                if ((axesAvail & (1 << a)) != 0) {
                    axes[a] = DriverStationBackend.getStickAxis(port, a);
                }
            }
            t.log("AxesAvailable", axesAvail);
            t.log("AxisValues", axes);

            int povsAvail = DriverStationBackend.getStickPOVsAvailable(port);
            String[] povs = new String[32 - Integer.numberOfLeadingZeros(povsAvail)];
            for (int p = 0; p < povs.length; p++) {
                povs[p] =
                        (povsAvail & (1 << p)) != 0 ? DriverStationBackend.getStickPOV(port, p).name() : "";
            }
            t.log("POVsAvailable", povsAvail);
            t.log("POVValues", povs);

            for (int tp = 0; tp < DriverStationJNI.MAX_JOYSTICK_TOUCHPADS; tp++) {
                for (int f = 0; f < DriverStationJNI.MAX_JOYSTICK_TOUCHPAD_FINGERS; f++) {
                    if (!DriverStationBackend.getStickTouchpadFingerAvailable(port, tp, f)) {
                        continue;
                    }
                    TouchpadFinger finger = DriverStationBackend.getStickTouchpadFinger(port, tp, f);
                    String key = "Touchpad" + tp + "/Finger" + f + "/";
                    t.log(key + "Down", finger.down);
                    t.log(key + "X", finger.x);
                    t.log(key + "Y", finger.y);
                }
            }
        }
    }

    // ====================================================================================
    // NT clients (throttled), grouped by IP
    // ====================================================================================

    private static void logNTClients() {
        ConnectionInfo[] conns = NetworkTableInstance.getDefault().getConnections();
        Map<String, List<ConnectionInfo>> byIP = new HashMap<>();
        for (ConnectionInfo c : conns) {
            byIP.computeIfAbsent(c.remoteIp, ip -> new ArrayList<>()).add(c);
            s_knownClientIPs.add(c.remoteIp);
        }

        s_ntClients.log("Count", conns.length);
        s_ntClients.log("IPs", byIP.keySet().toArray(new String[0]));

        // IPs seen earlier but gone now are written as disconnected.
        for (String ip : s_knownClientIPs) {
            List<ConnectionInfo> list = byIP.getOrDefault(ip, List.of());
            TelemetryTable t = s_ntClients.getTable(ip);
            String[] ids = new String[list.size()];
            long[] ports = new long[list.size()];
            long[] protocols = new long[list.size()];
            for (int i = 0; i < list.size(); i++) {
                ids[i] = list.get(i).remoteId;
                ports[i] = list.get(i).remotePort;
                protocols[i] = list.get(i).protocolVersion;
            }
            t.log("Connected", !list.isEmpty());
            t.log("ClientCount", list.size());
            t.log("ClientIds", ids);
            t.log("RemotePorts", ports);
            t.log("ProtocolVersions", protocols);
        }
    }

    // ====================================================================================
    // Radio (polled on a background thread, written on the robot thread)
    // ====================================================================================

    private static void logRadio() {
        s_radio.log("Connected", s_radioConnected);
        s_radio.log("Status", s_radioStatusJson, "json");
    }

    private static void startBackgroundThread() {
        s_backgroundThread =
                new Thread(
                        () -> {
                            while (!Thread.currentThread().isInterrupted()) {
                                if (s_logRadio) {
                                    pollRadio();
                                }
                                pollWifi();
                                try {
                                    Thread.sleep((long) (RADIO_PERIOD_SECS * 1000));
                                } catch (InterruptedException e) {
                                    return;
                                }
                            }
                        },
                        "SystemLogger_Background");
        s_backgroundThread.setDaemon(true);
        s_backgroundThread.start();
    }

    private static void pollRadio() {
        int team = RobotController.getTeamNumber();
        if (team <= 0) {
            return;
        }
        String json = "";
        try {
            String url = "http://10." + (team / 100) + "." + (team % 100) + ".1/status";
            HttpURLConnection conn = (HttpURLConnection) new URI(url).toURL().openConnection();
            conn.setRequestMethod("GET");
            conn.setConnectTimeout(RADIO_TIMEOUT_MS);
            conn.setReadTimeout(RADIO_TIMEOUT_MS);
            try (InputStream in = conn.getInputStream()) {
                ByteArrayOutputStream out = new ByteArrayOutputStream();
                in.transferTo(out);
                json = out.toString(StandardCharsets.UTF_8).replaceAll("\\s+", "");
            } finally {
                conn.disconnect();
            }
        } catch (Exception e) {
            json = "";
        }
        s_radioStatusJson = json;
        s_radioConnected = !json.isEmpty();
    }

    // ====================================================================================
    // Systemcore WiFi (polled on the background thread)
    // ====================================================================================

    /**
     * Reads the Systemcore's own WiFi SSID and mode from Linux. WPILib has no API for this, so it asks
     * {@code iw} (works for both access-point and client mode), falling back to {@code iwgetid}
     * (client mode only). Leaves the last value in place if neither tool is available.
     */
    private static void pollWifi() {
        String iw = runCommand("iw", "dev", "wlan0", "info");
        if (iw != null) {
            String ssid = "";
            String mode = "";
            for (String line : iw.split("\\R")) {
                String l = line.trim();
                if (l.startsWith("ssid ")) {
                    ssid = l.substring(5).trim();
                } else if (l.startsWith("type ")) {
                    mode = l.substring(5).trim(); // "AP" or "managed"
                }
            }
            s_wifiSSID = ssid;
            s_wifiMode = mode;
            return;
        }
        String ssid = runCommand("iwgetid", "-r");
        if (ssid != null) {
            s_wifiSSID = ssid.trim();
            s_wifiMode = s_wifiSSID.isEmpty() ? "" : "managed";
        }
    }

    /** Runs a command with a 1 s timeout; returns its output, or null if it couldn't run. */
    private static String runCommand(String... command) {
        try {
            Process proc = new ProcessBuilder(command).redirectErrorStream(true).start();
            String output;
            try (InputStream in = proc.getInputStream()) {
                output = new String(in.readAllBytes(), StandardCharsets.UTF_8);
            }
            if (!proc.waitFor(1, java.util.concurrent.TimeUnit.SECONDS)) {
                proc.destroyForcibly();
                return null;
            }
            return proc.exitValue() == 0 ? output : null;
        } catch (IOException e) {
            return null; // tool not installed
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return null;
        }
    }

    // ====================================================================================
    // Controller system stats (throttled)
    // ====================================================================================

    private static void logSystem() {
        s_controller.log("SerialNumber", s_serialNumber);
        s_controller.log("RobotName", s_robotName);
        s_wifi.log("SSID", s_wifiSSID);
        s_wifi.log("Mode", s_wifiMode);
//        s_controller.log("CPUTemp", RobotController.getCPUTemp());

        Runtime rt = Runtime.getRuntime();
        s_jvm.log("HeapUsedMB", (rt.totalMemory() - rt.freeMemory()) / 1048576.0);
        s_jvm.log("HeapMaxMB", rt.maxMemory() / 1048576.0);

        File root = new File("/");
        s_controller.log("DiskFreeMB", root.getUsableSpace() / 1048576.0);
        s_controller.log("DiskTotalMB", root.getTotalSpace() / 1048576.0);

        if (!s_procAvailable) {
            return; // simulation on Windows/macOS
        }
        try {
            logCpuUsage();
            logMemory();
        } catch (IOException | RuntimeException e) {
            s_procAvailable = false;
        }
    }

    /** Overall CPU usage from the delta of the aggregate "cpu" line in /proc/stat. */
    private static void logCpuUsage() throws IOException {
        String line = Files.readAllLines(Path.of("/proc/stat")).get(0);
        String[] f = line.trim().split("\\s+");
        long total = 0;
        for (int i = 1; i < f.length; i++) {
            total += Long.parseLong(f[i]);
        }
        long idle = Long.parseLong(f[4]) + Long.parseLong(f[5]); // idle + iowait
        if (s_prevCpuTotal >= 0 && total > s_prevCpuTotal) {
            double busy = 1.0 - (double) (idle - s_prevCpuIdle) / (total - s_prevCpuTotal);
            s_dsMetrics.log("CPUUsagePercent", busy * 100.0);
        }
        s_prevCpuTotal = total;
        s_prevCpuIdle = idle;
    }

    /** System memory from /proc/meminfo (values there are in kB). */
    private static void logMemory() throws IOException {
        List<String> lines = Files.readAllLines(Path.of("/proc/meminfo"));
        long totalKb = -1;
        long availableKb = -1;
        for (String l : lines) {
            if (l.startsWith("MemTotal:")) {
                totalKb = parseKb(l);
            } else if (l.startsWith("MemAvailable:")) {
                availableKb = parseKb(l);
            }
            if (totalKb >= 0 && availableKb >= 0) {
                break;
            }
        }
        if (totalKb > 0 && availableKb >= 0) {
            s_controller.log("MemoryTotalMB", totalKb / 1024.0);
            s_controller.log("MemoryUsedMB", (totalKb - availableKb) / 1024.0);
            s_controller.log("MemoryUsagePercent", 100.0 * (totalKb - availableKb) / totalKb);
        }
    }

    private static long parseKb(String line) {
        return Long.parseLong(line.replaceAll("[^0-9]", ""));
    }
}