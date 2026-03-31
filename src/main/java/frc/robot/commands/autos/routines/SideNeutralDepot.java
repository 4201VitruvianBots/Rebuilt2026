// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.segments.IntakeAndShootFromDepot;
import frc.robot.commands.autos.segments.IntakeFromNeutralFirstPass;
import frc.robot.commands.autos.segments.ShootNearStart;
import frc.robot.commands.swerve.ResetGyroWithAngle;
import frc.team4201.lib.command.Auto;

public class SideNeutralDepot extends Auto {
  public SideNeutralDepot(AutoDependencies deps) {
    addCommands(
        new ResetGyroWithAngle(deps.swerveDrive, deps.vision::getVisionAngle),
        new ShootNearStart(deps, () -> false),
        new IntakeFromNeutralFirstPass(deps, true, () -> false),
        new ShootNearStart(deps, () -> false),
        new IntakeAndShootFromDepot(deps));
  }
}
