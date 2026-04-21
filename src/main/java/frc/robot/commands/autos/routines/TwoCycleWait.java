// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.AutoShoot;
import frc.robot.commands.autos.segments.IntakeFromNeutral;
import frc.robot.constants.ROBOT.TWO_CYCLE_PATH;
import frc.team4201.lib.command.Auto;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TwoCycleWait extends Auto {
  public TwoCycleWait(AutoDependencies deps, BooleanSupplier flipPath, boolean partnerFriendly, DoubleSupplier waitTime) {
    addCommands(
        new WaitCommand(waitTime.getAsDouble()),
        new IntakeFromNeutral(deps, flipPath, TWO_CYCLE_PATH.FIRST_PASS_CONSERVATIVE_RUSH),
        new AutoShoot(deps, 2.3).andThen(new PrintCommand("[AUTO] Finished shooting")),
        // getChoiceCommand(new InstantCommand(), new IntakeFromNeutral(deps, flipPath,
        // TWO_CYCLE_PATH.DEPOT), flipPath),
        // getChoiceCommand(new InstantCommand(), new AutoShoot(deps, 1.8), flipPath),
        new IntakeFromNeutral(deps, flipPath, TWO_CYCLE_PATH.SECOND_PASS_PARTNER_FRIENDLY),
        new AutoShoot(deps, 1.8).andThen(new PrintCommand("[AUTO] Finished shooting")));
  }
}
