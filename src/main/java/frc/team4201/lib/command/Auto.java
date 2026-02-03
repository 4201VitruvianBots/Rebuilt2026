package frc.team4201.lib.command;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team4201.lib.utils.TrajectoryUtils;
import java.util.function.BooleanSupplier;

// The default side for a path should be the depot side, thus it is only flipped if the autoSide
// input is on the outpost.
public abstract class Auto extends SequentialCommandGroup {
  protected final Command getPathCommand(
      TrajectoryUtils trajectoryUtils, PathPlannerPath path, BooleanSupplier flipToRight) {
    return new ProxyCommand(
        () -> {
          if (flipToRight.getAsBoolean()) {
            return trajectoryUtils.generatePPHolonomicCommand(path.mirrorPath());
          } else {
            return trajectoryUtils.generatePPHolonomicCommand(path);
          }
        });
  }

  // chooses between 2 paths depending on autoSide input
  protected final Command getChoiceCommand(
      TrajectoryUtils trajectoryUtils,
      PathPlannerPath choice1,
      PathPlannerPath choice2,
      BooleanSupplier autoSide) {
    return new ProxyCommand(
        () -> {
          if (autoSide.getAsBoolean()) {
            return trajectoryUtils.generatePPHolonomicCommand(choice1);
          } else {
            return trajectoryUtils.generatePPHolonomicCommand(choice2);
          }
        });
  }
}
