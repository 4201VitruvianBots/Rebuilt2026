// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.segments.IntakeAndShootFromDepot;
import frc.robot.commands.autos.segments.ShootNearStart;
import frc.team4201.lib.command.Auto;

public class SideNeutralDepotClimb extends Auto {
  public SideNeutralDepotClimb(AutoDependencies deps) {
    addCommands(
        new ShootNearStart(deps, () -> false),
        //   new IntakeFromNeutral(deps, true, () -> false),
        //   new ShootNearStart(deps, () -> false),
        new IntakeAndShootFromDepot(deps)
        // // TODO: add climb, old code here
        //   new ParallelCommandGroup(
        //       new Shoot(flywheel, hood, vision, swerveDrive).withTimeout(3),
        //       new Climb(climber, CLIMBER.CLIMBER_SETPOINT.LEVEL_ONE)
        //           .until(
        //               () ->
        //                   Math.abs(
        //                           climber.getHeight().in(Meter)
        //                               -
        // CLIMBER.CLIMBER_SETPOINT.LEVEL_ONE.getSetpoint().in(Meter))
        //                       < 0.1)),
        //   m_path6.andThen(() -> swerveDrive.setControl(stopRequest)),
        //   new Climb(climber, CLIMBER.CLIMBER_SETPOINT.START_POSITION)
        );
  }
}
