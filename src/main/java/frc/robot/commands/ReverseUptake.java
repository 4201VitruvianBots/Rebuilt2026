// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import frc.robot.constants.INDEXER.INDEXER_SPEED_1;
import frc.robot.constants.INDEXER.INDEXER_SPEED_2;
import frc.robot.constants.UPTAKE.UPTAKE_SPEED;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Uptake;

public class ReverseUptake extends ParallelCommandGroup {
  /** Creates a new ReverseUptake. */
  public ReverseUptake(Indexer indexer, Uptake uptake) {
    addCommands(
        (uptake != null)
            ? uptake.percentCommand(UPTAKE_SPEED.INTAKEREVERSING.get())
            : new InstantCommand(),
        (indexer != null)
            ? indexer.command(INDEXER_SPEED_1.FREEING, INDEXER_SPEED_2.FREEING)
            : new InstantCommand());
  }
}
