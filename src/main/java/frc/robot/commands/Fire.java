// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.INDEXER.INDEXER_SPEED_1;
import frc.robot.constants.INDEXER.INDEXER_SPEED_2;
import frc.robot.constants.INTAKE.ROLLERS.INTAKE_STATE;
import frc.robot.constants.UPTAKE.UPTAKE_SPEED;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;

// *insert fire emoji*
public class Fire extends ParallelCommandGroup {
  /** Creates a new Fire. */
  public Fire(Intake intake, Indexer indexer, Uptake uptake) {
    addCommands(
        (intake != null) ? intake.commandIntakeState(INTAKE_STATE.SHOOTING) : new InstantCommand(),
        (indexer != null) ? indexer.command(INDEXER_SPEED_1.INDEXING, INDEXER_SPEED_2.INDEXING) : new InstantCommand(),
        (uptake != null) ? uptake.percentCommand(UPTAKE_SPEED.SHOOTING.get()) : new InstantCommand());
  }
}
