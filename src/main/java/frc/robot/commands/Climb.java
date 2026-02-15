package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CLIMBER.*;
import frc.robot.subsystems.Climber;

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
    if (m_climber.isHoldingRobot()) {
      m_climber.setPIDSlot(HOLDING_ROBOT.slot);
      m_climber.setDesiredPositionAndMotionMagicConfigs(
          m_setpoint.getSetpoint(),
          HOLDING_ROBOT.motionMagicCruiseVelocity,
          HOLDING_ROBOT.motionMagicAcceleration,
          HOLDING_ROBOT.motionMagicJerk);
    } else {
      m_climber.setPIDSlot(NOT_HOLDING_ROBOT.slot);
      m_climber.setDesiredPositionAndMotionMagicConfigs(
          m_setpoint.getSetpoint(),
          NOT_HOLDING_ROBOT.motionMagicCruiseVelocity,
          NOT_HOLDING_ROBOT.motionMagicAcceleration,
          NOT_HOLDING_ROBOT.motionMagicJerk);
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
