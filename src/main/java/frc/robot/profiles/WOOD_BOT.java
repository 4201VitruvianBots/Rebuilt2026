package frc.robot.profiles;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.profiles.REBUILT_ROBOT.SUBSYSTEMS.*;

import edu.wpi.first.epilogue.Logged;
import frc.robot.generated.WoodBotConstants;
import java.util.List;

public class WOOD_BOT extends REBUILT_ROBOT {

  public static void init() {
    robotId = ROBOT_ID.WOOD_BOT;
    robotName = robotId.getName();
    logMode = Logged.Importance.INFO;
    swerveDrive = WoodBotConstants.createDrivetrain();

    subsystems.addAll(List.of(SWERVE_DRIVE, FLYWHEEL, INTAKE));

    WIDTH = Inches.of(23.0);
    LENGTH = Inches.of(33.0);
    BUMPER_THICKNESS = Inches.of(2.75);
  }
}
