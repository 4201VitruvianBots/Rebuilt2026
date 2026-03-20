package frc.team4201.lib.hardwareMonitor.configs;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import frc.team4201.lib.command.SwerveSubsystem;

public abstract class BASE_ROBOT {
  // Robot Dimensions
  public static Distance WIDTH = Inches.zero();
  public static Distance LENGTH = Inches.zero();
  public static Distance BUMPER_THICKNESS = Inches.zero();

  // Software configs
  public static String robotName = "";
  public static ROBOT_ID robotId = ROBOT_ID.SIM;
  public static Logged.Importance logMode = Logged.Importance.INFO;
  public static SwerveSubsystem swerveDrive;

  public static void init() {}

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
    V2("NULL"),
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
}
