package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;

public class ROBOT {
  public static final boolean useSysID = false;

  public static Logged.Importance logMode = Logged.Importance.INFO;
  public static ROBOT_ID robotID = ROBOT_ID.SIM;

  public enum CONTROL_MODE {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  public enum SUPERSTRUCTURE_STATES {}

  public enum ROBOT_ID {
    // Robot Serial Numbers (2023-2025)
    // ALPHABOT - 030cbc95
    // LOCHNESS(V2) - 032381FB
    // LOCHNESS(V3) - 032398ED
    // FORTE - 030cbc95
    // GRIDLOCK - 0306ce62
    // BOBOT - 030e6a97

    // 030cbcf0 - 23-1 Rio 2.0
    // 030cbd1c - 23-2 Rio 1.0 (in the green bins)
    // 0310d915 - 23-3 Rio 1.0 (Doesn't work right - Sheraz)

    // Robot Serial Numbers (2026)
    WOOD_BOT("030cbc95"), // Rio 1.0
    V1("030cbcf0"),

    SIM("");

    private final String value;

    ROBOT_ID(final String value) {
      this.value = value;
    }

    public String getSerial() {
      return value;
    }

    public String getName() {
      return name();
    }

    @Override
    public String toString() {
      return value;
    }

    public static ROBOT_ID fromSerial(String serial) {
      for (ROBOT_ID id : ROBOT_ID.values()) {
        if (id.value.equalsIgnoreCase(serial)) return id;
      }
      throw new IllegalArgumentException("Serial " + serial + " not defined in ROBOT_ID!");
    }
  }

  public static void initWoodBot() {
    robotID = ROBOT_ID.WOOD_BOT;
  }

  public static final Distance ROBOTWIDTH = Inches.of(23.0);
  public static final Distance ROBOTLENGTH = Inches.of(33.0);
  public static final Distance BUMPERTHICKNESS = Inches.of(2.75);

  public static void initSim() {
    logMode = Logged.Importance.DEBUG;
    robotID = ROBOT_ID.SIM;
  }

  public static void initV1() {
    robotID = ROBOT_ID.V1;
  }

  public static void initConstants() {
    var alert = new Alert("Initializing Robot Constants...", AlertType.kInfo);

    try {
      switch (ROBOT_ID.fromSerial(RobotController.getSerialNumber())) {
        case WOOD_BOT -> initWoodBot();
        case V1 -> initV1();
        case SIM -> {
          initSim();
          System.out.print(
              """
                          !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                          !!! WARNING: This will put logging in debug mode                     !!!
                          !!!          and almost certainly crash the real robot!              !!!
                          !!! IF YOU ARE SEEING THIS IN THE DS CONSOLE, YOUR ROBOT WILL CRASH! !!!
                          !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                          """);
        }
      }
      alert.setText("Setting Robot Constants for " + robotID.getName());
    } catch (IllegalArgumentException e) {
      alert =
          new Alert(
              "WARN: Robot Serial Not Recognized! Current roboRIO Serial: "
                  + RobotController.getSerialNumber(),
              AlertType.kWarning);
    }
    alert.set(true);
  }
}
