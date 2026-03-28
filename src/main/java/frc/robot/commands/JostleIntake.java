// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;
import frc.robot.constants.INTAKE.PIVOT;
import frc.robot.constants.INTAKE.PIVOT.PIVOT_SETPOINT;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JostleIntake extends Command {

  private final IntakePivot m_intakePivot;
  private Timer m_sineWaveTimer = new Timer();
  /** Creates a new JostleIntake. */
  public JostleIntake(IntakePivot intakePivot) {
    m_intakePivot = intakePivot;

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_sineWaveTimer.reset();
    m_sineWaveTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Angle angle = Degrees.of(PIVOT_SETPOINT.INTAKING.getAngle().abs(Degrees) * Math.cos((PIVOT.pivotCycleTime.abs(Seconds) * m_sineWaveTimer.get() * Math.PI * 2)) + PIVOT_SETPOINT.INTAKING.getAngle().abs(Degrees));
    m_intakePivot.setAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_sineWaveTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
