package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.INDEXER;
import frc.robot.constants.UPTAKE;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Uptake;

public class Fire extends Command {
  private Uptake m_uptake;
  private Indexer m_indexer;

  public Fire(Uptake uptake, Indexer indexer) {
    m_uptake = uptake;
    m_indexer = indexer;

    addRequirements(uptake, indexer);
    SmartDashboard.putData(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexer.command(INDEXER.INDEXER_SPEED_1.INDEXING, INDEXER.INDEXER_SPEED_2.INDEXING);
    m_uptake.command(UPTAKE.UPTAKE_SPEED.INTAKE_RUN);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.command(INDEXER.INDEXER_SPEED_1.ZERO, INDEXER.INDEXER_SPEED_2.ZERO);
    m_uptake.command(UPTAKE.UPTAKE_SPEED.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
