// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.PrintCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.AutoShoot;
import frc.robot.commands.autos.segments.IntakeFromNeutral;
import frc.robot.constants.ROBOT.TWO_CYCLE_PATH;
import frc.team4201.lib.command.Auto;
import java.util.function.BooleanSupplier;

public class TwoCycleInsideOutConservativeRush extends Auto {
  public TwoCycleInsideOutConservativeRush(
      AutoDependencies deps, BooleanSupplier flipPath, boolean partnerFriendly) {
    addCommands(
        new IntakeFromNeutral(
            deps, flipPath, TWO_CYCLE_PATH.FIRST_PASS_INSIDE_OUT_CONSERVATIVE_RUSH),
        new AutoShoot(deps, 2.6).andThen(new PrintCommand("[AUTO] Finished shooting")),
        getChoiceCommand(
            new InstantCommand(),
            new IntakeFromNeutral(deps, flipPath, TWO_CYCLE_PATH.DEPOT),
            flipPath),
        getChoiceCommand(new InstantCommand(), new AutoShoot(deps, 1.8), flipPath),
        new IntakeFromNeutral(
            deps,
            flipPath,
            partnerFriendly
                ? TWO_CYCLE_PATH.SECOND_PASS_PARTNER_FRIENDLY
                : TWO_CYCLE_PATH.SECOND_PASS),
        new AutoShoot(deps, 1.8).andThen(new PrintCommand("[AUTO] Finished shooting")));
  }
}
