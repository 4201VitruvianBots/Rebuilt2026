// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.INDEXER.INDEXER_SPEED_1;
import frc.robot.constants.INTAKE.PIVOT.PIVOT_SETPOINT;
import frc.robot.constants.INTAKE.ROLLERS.INTAKE_STATE;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakePivot;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;

public class IntakeCommand extends ParallelCommandGroup {
  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, IntakePivot intakePivot, Indexer indexer) {
    addCommands(
        (intake != null) ? intake.commandIntakeState(INTAKE_STATE.INTAKING) : new InstantCommand(),
        (indexer != null) ? indexer.command(INDEXER_SPEED_1.FREEING) : new InstantCommand(),
        (intakePivot != null)
            ? intakePivot.command(PIVOT_SETPOINT.INTAKING)
            : new InstantCommand());
  }
}
