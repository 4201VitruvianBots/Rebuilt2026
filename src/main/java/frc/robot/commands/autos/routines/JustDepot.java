// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.AutoShoot;
import frc.robot.commands.autos.segments.IntakeFromNeutral;
import frc.robot.commands.autos.segments.IntakeFromNeutralNoCross;
import frc.robot.constants.ROBOT.TWO_CYCLE_PATH;
import frc.team4201.lib.command.Auto;
import java.util.function.BooleanSupplier;

import org.wpilib.command2.WaitCommand;

public class JustDepot extends Auto {
  public JustDepot(AutoDependencies deps, BooleanSupplier flipPath) {
    addCommands(
        new IntakeFromNeutralNoCross(deps, flipPath, TWO_CYCLE_PATH.DEPOT), new AutoShoot(deps, 10.0));
  }
}
