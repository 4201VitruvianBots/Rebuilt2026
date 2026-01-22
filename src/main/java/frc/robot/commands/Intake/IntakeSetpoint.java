// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INTAKEMOTORS.PIVOT.PIVOT_SETPOINT;
import frc.robot.subsystems.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeSetpoint extends Command {
  private final IntakePivot m_IntakePivot;
  private final PIVOT_SETPOINT m_setpoint;

  /** Creates a new IntakeSetpoint. */
  public IntakeSetpoint(IntakePivot intakePivot, PIVOT_SETPOINT setpoint) {
    m_IntakePivot = intakePivot;
    m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakePivot.setAngle(m_setpoint.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakePivot.setAngle(PIVOT_SETPOINT.STOWED.getAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
