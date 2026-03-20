package frc.team4201.lib.hardwareMonitor.annotations.devices;

import static com.ctre.phoenix6.StatusCode.*;
import static com.ctre.phoenix6.signals.MagnetHealthValue.*;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.wpilibj.RobotBase;
import frc.team4201.lib.hardwareMonitor.HardwareMonitor.HEALTH_STATUS;
import frc.team4201.lib.hardwareMonitor.annotations.BaseDeviceMonitor;
import frc.team4201.lib.hardwareMonitor.annotations.HealthStatusFor;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredDevice;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

@HealthStatusFor(CANcoder.class)
public class CANcoderStatus extends BaseDeviceMonitor<CANcoder> {
  static Map<HEALTH_STATUS, EnumSet<StatusCode>> STATUS_CODE_BITMASKS =
      new LinkedHashMap<>(
          Map.ofEntries(
              Map.entry(
                  HEALTH_STATUS.BAD,
                  EnumSet.of(
                      GeneralError,
                      BufferFailure,
                      NoDevicesOnBus,
                      AppTooOld,
                      DeviceDidNotRespondToDiagReq,
                      DeviceIsNull,
                      UsingProFeatureOnUnlicensedDevice)),
              Map.entry(
                  HEALTH_STATUS.DEGRADED,
                  EnumSet.of(GeneralWarning, CanMessageStale, RxTimeout, TxTimeout))));
  static EnumSet<StatusCode> STATUS_CODE_IGNORE_BITMASK = EnumSet.of(OK);
  static Map<HEALTH_STATUS, EnumSet<MagnetHealthValue>> MAGNET_HEALTH_OUTPUT_BITMASK =
      new LinkedHashMap<>(
          Map.ofEntries(
              Map.entry(HEALTH_STATUS.BAD, EnumSet.of(Magnet_Orange)),
              Map.entry(HEALTH_STATUS.DEGRADED, EnumSet.of(Magnet_Invalid, Magnet_Red))));

  public CANcoderStatus() {
    super(CANcoder.class);
  }

  public LinkedList<BaseStatusSignal> getSignals(
      CANcoder cancoder, MonitoredDevice.DEVICE_TYPE deviceType) {
    var signals =
        new LinkedList<BaseStatusSignal>(
            List.of(cancoder.getMagnetHealth(), cancoder.getAbsolutePosition()));

    if (deviceType == MonitoredDevice.DEVICE_TYPE.PRIMARY) {
      var primarySignals =
          new LinkedList<BaseStatusSignal>(List.of(cancoder.getPosition(), cancoder.getVelocity()));

      signals.addAll(primarySignals);
    }

    if (RobotBase.isSimulation()) {
      BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(250), signals);
    } else if (deviceType == MonitoredDevice.DEVICE_TYPE.PRIMARY) {
      BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(100), signals);
    } else {
      BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), signals);
    }

    if (!RobotBase.isSimulation()) cancoder.optimizeBusUtilization();

    return signals;
  }

  @Override
  public HEALTH_STATUS checkHealth(CANcoder cancoder) {
    AtomicReference<HEALTH_STATUS> motorHealthStatus = new AtomicReference<>(HEALTH_STATUS.HEALTHY);

    var magnetHealth = cancoder.getMagnetHealth();
    if (magnetHealth.getStatus() != OK) {
      STATUS_CODE_BITMASKS.forEach(
          (s, v) -> {
            if (v.contains(magnetHealth.getStatus())) {
              motorHealthStatus.set(s);
            }
          });
    } else {
      MAGNET_HEALTH_OUTPUT_BITMASK.forEach(
          (s, v) -> {
            if (v.contains(magnetHealth.getValue())) {
              motorHealthStatus.set(s);
            }
          });
    }

    // TODO: To implement
    var faultStatus = cancoder.getFaultField();

    var stickFaultStatus = cancoder.getStickyFaultField();

    return motorHealthStatus.get();
  }
}
