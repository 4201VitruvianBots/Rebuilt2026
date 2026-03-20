// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.INTAKE.PIVOT.PIVOT_SETPOINT;
import frc.robot.constants.INTAKE.ROLLERS.INTAKE_SPEED;
import frc.robot.constants.UPTAKE.UPTAKE_SPEED;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Uptake;

public class IntakeCommand extends ParallelCommandGroup {
  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, IntakePivot intakePivot, Uptake uptake) {
    addCommands(
        (intake != null) ? intake.command(INTAKE_SPEED.INTAKING) : new InstantCommand(),
        (uptake != null)
            ? uptake.percentCommand(UPTAKE_SPEED.INTAKEREVERSING.get())
            : new InstantCommand(),
        (intakePivot != null)
            ? intakePivot.command(PIVOT_SETPOINT.INTAKING)
            : new InstantCommand());
  }
}
