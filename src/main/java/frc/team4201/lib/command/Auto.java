package frc.team4201.lib.command;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team4201.lib.utils.TrajectoryUtils;

import java.util.function.BooleanSupplier;

public abstract class Auto extends SequentialCommandGroup {
    protected final Command getPathCommand(TrajectoryUtils trajectoryUtils, PathPlannerPath path, BooleanSupplier flipToRight) {
        return new ProxyCommand(()->{
            if(flipToRight.getAsBoolean()) {
                return trajectoryUtils.generatePPHolonomicCommand(path.mirrorPath());
            } else {
                return trajectoryUtils.generatePPHolonomicCommand(path);
            }
        });
    }
}
