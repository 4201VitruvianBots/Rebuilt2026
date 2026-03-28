// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.segments.IntakeFromNeutralFirstPass;
import frc.robot.commands.autos.segments.IntakeFromNeutralSecondPass;
import frc.robot.commands.autos.segments.ShootNearStart;
import frc.team4201.lib.command.Auto;
import java.util.function.BooleanSupplier;

public class SideNeutralTwice extends Auto {
  public SideNeutralTwice(AutoDependencies deps, BooleanSupplier flipPath, boolean rushCenter) {

    addCommands(
        rushCenter ? new InstantCommand() : new ShootNearStart(deps, flipPath),
        new IntakeFromNeutralFirstPass(deps, !rushCenter, flipPath),
        new ShootNearStart(deps, flipPath),
        new IntakeFromNeutralSecondPass(deps, true, flipPath),
        new ShootNearStart(deps, flipPath));
  }
}
