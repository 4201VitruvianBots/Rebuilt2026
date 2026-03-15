// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.segments.IntakeFromNeutral;
import frc.robot.commands.autos.segments.ShootNearStart;
import frc.team4201.lib.command.Auto;
import java.util.function.BooleanSupplier;

public class SideNeutralClimb extends Auto {
  public SideNeutralClimb(AutoDependencies deps, BooleanSupplier flipPath) {
    addCommands(
        new ShootNearStart(deps, flipPath),
        new IntakeFromNeutral(deps, true, flipPath),
        new ShootNearStart(deps, flipPath)
        // TODO: add climb
        );
  }
}
