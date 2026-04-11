// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.INTAKE.PIVOT;
import frc.robot.constants.INTAKE.PIVOT.PIVOT_SETPOINT;
import frc.robot.constants.INTAKE.ROLLERS.INTAKE_SPEED;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JostleIntake extends Command {

  private final IntakePivot m_intakePivot;
   private final Intake m_intake;
  private Timer m_sineWaveTimer = new Timer();

  /** Creates a new JostleIntake. */
  public JostleIntake(IntakePivot intakePivot) {
   this(intakePivot, null);
  }
  /** Creates a new JostleIntake. */
  public JostleIntake(IntakePivot intakePivot,Intake intake) {
    m_intakePivot = intakePivot;
    m_intake = intake;
    if (m_intake == null)
    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(intakePivot) ;
      else
      addRequirements(intakePivot, intake);
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
    // https://www.desmos.com/calculator/agh8canlyu
    Angle angle =
        Degrees.of(
            PIVOT.maxAngle.in(Degrees)
                    * Math.cos(
                        (m_sineWaveTimer.get() * Math.PI / PIVOT.pivotCycleTime.in(Seconds)))
                + PIVOT.maxAngle.abs(Degrees));
    m_intakePivot.setAngle(angle);
    if (m_intake != null)
    m_intake.setOutputPercent(INTAKE_SPEED.INTAKING.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_sineWaveTimer.stop();
    m_intakePivot.setAngle(PIVOT_SETPOINT.INTAKING.getAngle());
        if (m_intake != null)
    m_intake.setOutputPercent(INTAKE_SPEED.ZERO.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
