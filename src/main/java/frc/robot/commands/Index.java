package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INDEXER.INDEXER_SPEED;
import frc.robot.subsystems.Indexer;

public class Index extends Command {

  private final Indexer m_indexer;

  private final INDEXER_SPEED m_speed;

  /** Creates a new Index. */
  public Index(Indexer indexer, INDEXER_SPEED speed) {
    m_indexer = indexer;
    m_speed = speed;

    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexer.setSpeed(m_speed.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
