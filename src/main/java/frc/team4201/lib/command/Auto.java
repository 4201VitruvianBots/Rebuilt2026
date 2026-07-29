package frc.team4201.lib.command;

import com.pathplanner.lib.path.PathPlannerPath;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Set;
import java.util.function.BooleanSupplier;

// The default side for a path should be the depot side, thus it is only flipped if the autoSide
// input is on the outpost.
public abstract class Auto extends SequentialCommandGroup {
  protected final Command getPathCommand(
      CommandSwerveDrivetrain swerveDrive, PathPlannerPath path, BooleanSupplier flipToRight) {
    return Commands.defer(
        () -> {
          if (flipToRight.getAsBoolean()) {
            return swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(path.mirrorPath());
          } else {
            return swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(path);
          }
        },
        Set.of(swerveDrive));
  }

  // chooses between 2 paths depending on autoSide input
  protected final Command getChoiceCommand(
      CommandSwerveDrivetrain swerveDrive,
      PathPlannerPath choice1,
      PathPlannerPath choice2,
      BooleanSupplier autoSide) {
    return Commands.defer(
        () -> {
          if (autoSide.getAsBoolean()) {
            return swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(choice1);
          } else {
            return swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(choice2);
          }
        },
        Set.of(swerveDrive));
  }
}
