package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.INDEXER;
import frc.robot.constants.UPTAKE;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Uptake;

public class Fire extends ParallelCommandGroup {
  public Fire(Uptake uptake, Indexer indexer) {
      addCommands(
        indexer.command(INDEXER.INDEXER_SPEED_1.INDEXING, INDEXER.INDEXER_SPEED_2.INDEXING),
        uptake.percentCommand(UPTAKE.UPTAKE_SPEED.UPTAKING.get())
      );

        addRequirements(uptake, indexer);
        SmartDashboard.putData(this);
  }
}
