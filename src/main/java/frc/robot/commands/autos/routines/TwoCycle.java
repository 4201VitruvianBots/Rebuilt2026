// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.AutoShoot;
import frc.robot.commands.autos.segments.IntakeFromNeutral;
import frc.robot.constants.ROBOT.TWO_CYCLE_PATH;
import frc.team4201.lib.command.Auto;
import java.util.function.BooleanSupplier;

public class TwoCycle extends Auto {
  public TwoCycle(AutoDependencies deps, BooleanSupplier flipPath, boolean partnerFriendly) {
    addCommands(
        new IntakeFromNeutral(deps, flipPath, TWO_CYCLE_PATH.FIRST_PASS_RUSH),
        new AutoShoot(deps, 2.6).andThen(new PrintCommand("[AUTO] Finished shooting")),
        new IntakeFromNeutral(deps, flipPath, TWO_CYCLE_PATH.SECOND_PASS_PARTNER_FRIENDLY),
        new AutoShoot(deps, 1.8).andThen(new PrintCommand("[AUTO] Finished shooting")));
  }
}
