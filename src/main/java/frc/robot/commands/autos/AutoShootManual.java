// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Fire;

// Begins firing once the shooter is up to speed, and continues firing for a given duration. Used in
// auto routines to fire shots while running paths.
// Does not use vision. Used as a backup for CenterPreload if vision fails.
public class AutoShootManual extends ParallelDeadlineGroup {
  /** Creates a new AutoShoot. */
  public AutoShootManual(AutoDependencies deps, double fireDurationSeconds) {
    super(
        Commands.waitUntil(
                () ->
                    (deps.flywheel.isAtRPMsetpoint()
                        && deps.hood.atSetpoint()
                        ))
            .withTimeout(fireDurationSeconds)
            .andThen(
                new Fire(deps.intake, deps.indexer, deps.uptake).withTimeout(fireDurationSeconds)));
    addCommands(deps.flywheel.manualAgainstHubCommand(), deps.hood.manualAgainstHubCommand());
  }
}
