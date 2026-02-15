// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.RunUptake;
import frc.robot.constants.INDEXER.INDEXER_SPEED;
import frc.robot.constants.INTAKE;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Uptake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAll extends ParallelCommandGroup {
  /** Creates a new FullIntake. */
  public IntakeAll(Intake intake, IntakePivot intakePivot, Indexer indexer, Uptake uptake) {

    addCommands(
        new RunIntake(intake, INTAKE.ROLLERS.INTAKE_SPEED.INTAKING),
        new Index(indexer, INDEXER_SPEED.INDEXING, INDEXER_SPEED.INDEXING),
        new RunUptake(uptake),
        new IntakeSetpoint(intakePivot, INTAKE.PIVOT.PIVOT_SETPOINT.INTAKING));
  }
}
