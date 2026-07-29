// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import org.wpilib.command2.InstantCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.segments.IntakeFromNeutral;
import frc.robot.commands.autos.segments.ShootNearStart;
import frc.team4201.lib.command.Auto;
import java.util.function.BooleanSupplier;

public class SideNeutralTwice extends Auto {
  public SideNeutralTwice(AutoDependencies deps, BooleanSupplier flipPath, boolean rushCenter) {

    addCommands(
        rushCenter ? new InstantCommand() : new ShootNearStart(deps, flipPath),
        new IntakeFromNeutral(deps, !rushCenter, flipPath),
        new ShootNearStart(deps, flipPath),
        new IntakeFromNeutral(deps, true, flipPath),
        new ShootNearStart(deps, flipPath));
  }
}
