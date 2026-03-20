package frc.team4201.lib.hardwareMonitor;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.profiles.SIM;
import frc.robot.profiles.V1;
import frc.robot.profiles.WOOD_BOT;
import frc.team4201.lib.hardwareMonitor.configs.BASE_ROBOT;
import java.util.Objects;

public class HardwareManager {
  private static Alert hardwareAlerts = new Alert("", Alert.AlertType.kError);
  // private final Map<String, ? extends IBaseDevice> subsystemDeviceMap = new LinkedHashMap<>();
  private static double startBootTimestamp = 0;
  private static double startSyncTimestamp = 0;
  private static double startCheckTimestamp = 0;
  private static double endSyncTimestamp = 0;

  private static double lastCheckDuration = 0;

  public static void initialize() {
    startBootTimestamp = Timer.getFPGATimestamp();
    loadRobotConfig();
    // Init all CAN devices here:
    initializeHardware(
        DriverStation.isFMSAttached()
            && (DriverStation.isAutonomousEnabled() || DriverStation.isTeleopEnabled()));
  }

  private static void loadRobotConfig() {
    System.out.println("HardwareManager::loadRobotConfig() [INFO] Checking the roboRIO serial...");
    var robotSerial = RobotController.getSerialNumber();
    var robotId = BASE_ROBOT.ROBOT_ID.fromSerial(robotSerial);

    switch (robotId) {
      case SIM -> SIM.init();
      case WOOD_BOT -> WOOD_BOT.init();
      case V1 -> V1.init();
    }

    if (Objects.equals(BASE_ROBOT.robotName, "")) {
      hardwareAlerts =
          new Alert(
              "[ERROR] Unknown Robot Serial detected! Defaulting to V1", Alert.AlertType.kError);
      hardwareAlerts.set(true);
      V1.init();
    }
    System.out.printf(
        "HardwareManager::loadRobotConfig() [INFO] RobotConfig loaded for %s(%s)\n",
        robotId.getName(), robotSerial);
  }

  public static void initializeHardware() {
    initializeHardware(false);
  }

  public static void initializeHardware(boolean fastboot) {
    System.out.println("HardwareManager::initializeHardware() [INFO] Initializing hardware...");

    // Always boot drivetrain first

    if (fastboot) {
      // TODO: Use threading for this
      // Subsystem priority order intake->indexer->kicker->shooter->climber

    } else {
      HardwareMonitor.initAllSubsystems();
    }
    System.out.printf(
        "HardwareManager::initializeHardware() [INFO] Hardware initialized in %.3f Seconds!\n",
        Timer.getFPGATimestamp() - startBootTimestamp);
  }

  public static void checkHealth() {
    // Return/get allStatus
    startCheckTimestamp = Timer.getFPGATimestamp();
    HardwareMonitor.checkRobotHealth();
    updateHardwareManagerAlert();
    endSyncTimestamp = Timer.getFPGATimestamp();
    lastCheckDuration = endSyncTimestamp - startCheckTimestamp;
  }

  public static void updateHardwareManagerAlert() {
    // TODO: To implement

  }

  public void clearErrors() {
    // TODO: To implement
  }
}
