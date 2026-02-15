package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.UPTAKE;
import frc.robot.subsystems.Uptake;

public class RunUptake extends Command {

  private final Uptake m_uptake;

  /** Creates a new RunUptake. */
  public RunUptake(Uptake uptake) {
    m_uptake = uptake;

    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_uptake.setVelocitySetpoint(UPTAKE.UPTAKE_SPEED.UPTAKING.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_uptake.setPercentOutput(0.0);
    m_uptake.setVelocitySetpoint(UPTAKE.UPTAKE_SPEED.IDLE.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
