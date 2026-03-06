package frc.team4201.lib.command;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Set;
import java.util.function.BooleanSupplier;

// The default side for a path should be the depot side, thus it is only flipped if the autoSide
// input is on the outpost.
public abstract class Auto extends SequentialCommandGroup {
  protected final Command getPathCommand(
      CommandSwerveDrivetrain swerveDrive, PathPlannerPath path, BooleanSupplier flipToRight) {
    return Commands.deferredProxy(
        () -> {
          if (flipToRight.getAsBoolean()) {
            return swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(path.mirrorPath());
          } else {
            return swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(path);
          }
        });
  }

  // chooses between 2 paths depending on autoSide input
  protected final Command getChoiceCommand(
      CommandSwerveDrivetrain swerveDrive,
      PathPlannerPath choice1,
      PathPlannerPath choice2,
      BooleanSupplier autoSide) {
    return Commands.deferredProxy(
        () -> {
          if (autoSide.getAsBoolean()) {
            return swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(choice1);
          } else {
            return swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(choice2);
          }
        });
  }
}
