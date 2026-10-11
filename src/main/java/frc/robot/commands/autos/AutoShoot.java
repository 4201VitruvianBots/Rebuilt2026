// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import frc.robot.commands.Fire;
import frc.robot.commands.JostleIntake;
import frc.robot.commands.Shoot;
import org.wpilib.command2.*;

// Begins firing once the shooter is up to speed, and continues firing for a given duration. Used in
// auto routines to fire shots while running paths.
public class AutoShoot extends ParallelDeadlineGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(AutoDependencies deps, double fireDurationSeconds) {
    super(
        Commands.waitUntil(
                () ->
                    (deps.flywheel.isAtRPMsetpoint()
                        && deps.hood.atSetpoint()
                        && deps.vision.isOnTarget()))
            .withTimeout(1.0)
            .andThen(
                new ParallelDeadlineGroup(
                    new ConditionalCommand(
                        new InstantCommand(),
                        new Fire(deps.indexer, deps.intakePivot)
                            .withTimeout(fireDurationSeconds),
                        deps.vision::isInNeutralSector))));
    addCommands(new Shoot(deps.flywheel, deps.hood, deps.vision, deps.swerveDrive));
  }
}
