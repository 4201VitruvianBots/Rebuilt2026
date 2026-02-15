// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.INDEXER.INDEXER_SPEED;
import frc.robot.constants.UPTAKE.UPTAKE_SPEED;
import frc.robot.constants.INTAKE;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Uptake;

public class IntakeCommand extends ParallelCommandGroup {
  /** Creates a new IntakeAll. */
  public IntakeCommand(Intake intake, IntakePivot intakePivot, Indexer indexer, Uptake uptake) {
    addCommands(
        intake.command(INTAKE.ROLLERS.INTAKE_SPEED.INTAKING),
        indexer.command(INDEXER_SPEED.INDEXING, INDEXER_SPEED.INDEXING),
        uptake.command(UPTAKE_SPEED.UPTAKING),
        intakePivot.command(INTAKE.PIVOT.PIVOT_SETPOINT.INTAKING));
  }
}
