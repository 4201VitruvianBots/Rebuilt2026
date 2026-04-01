// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.AutoShoot;
import frc.robot.commands.autos.segments.IntakeFromNeutral;
import frc.team4201.lib.command.Auto;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.PrintCommand;

public class TwoCycle extends Auto {
  public TwoCycle(AutoDependencies deps, BooleanSupplier flipPath) {
    addCommands(
        new IntakeFromNeutral(deps, flipPath, false),
        new AutoShoot(deps, 3.0)
              .andThen(new PrintCommand("[AUTO] Finished shooting")),
        new IntakeFromNeutral(deps, flipPath, true),
        new AutoShoot(deps, 1.8)
              .andThen(new PrintCommand("[AUTO] Finished shooting")));
  }
}
