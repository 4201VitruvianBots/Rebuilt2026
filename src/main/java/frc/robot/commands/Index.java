// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INDEXERMOTORS;
import frc.robot.Constants.INDEXERMOTORS.INDEXERSPEED;
import frc.robot.subsystems.Indexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Index extends Command {

  private final Indexer m_indexer;

  private final INDEXERSPEED m_speed;

  
  /** Creates a new Index. */
  public Index(Indexer indexer, INDEXERSPEED speed) {
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

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
