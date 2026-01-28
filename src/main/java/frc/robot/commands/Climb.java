// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports:
package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.CLIMBER.CLIMBER_SETPOINT;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  // Creates a new Climb Command
  private final Climber m_climber;

  private CLIMBER_SETPOINT m_setpoint;

  public Climb(Climber climber, CLIMBER_SETPOINT setpoint) {
    m_climber = climber;
    m_setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climber.getStatorCurrent().in(Amps) < CLIMBER.kHoldingRobotThreshold) {
      m_climber.setPIDSlot(0);
      m_climber.setDesiredPositionAndMotionMagicConfigs(
          m_setpoint.getSetpoint(),
          CLIMBER.motionMagicCruiseVelocitynoRobot,
          CLIMBER.motionMagicAccelerationnoRobot,
          0.0);
      System.out.println("No Robot");
    } else {
      m_climber.setPIDSlot(1);
      m_climber.setDesiredPositionAndMotionMagicConfigs(
          m_setpoint.getSetpoint(),
          CLIMBER.motionMagicCruiseVelocityRobot,
          CLIMBER.motionMagicAccelerationRobot,
          0.0);
      System.out.println("With Robot");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.holdClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
