package frc.robot.profiles;

import frc.team4201.lib.hardwareMonitor.configs.BASE_ROBOT;
import java.util.LinkedList;

public class REBUILT_ROBOT extends BASE_ROBOT {
  // Hardware configs
  public static LinkedList<SUBSYSTEMS> subsystems = new LinkedList<>();

  public enum SUBSYSTEMS {
    SWERVE_DRIVE,
    FLYWHEEL,
    UPTAKE,
    INDEXER,
    INTAKE,
    HOOD,
    INTAKE_PIVOT,
    LEDS,
    CONTROLS,
    VISION,
    //    CLIMBER
  }
}
