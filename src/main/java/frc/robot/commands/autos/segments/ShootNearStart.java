// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.segments;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Fire;
import frc.robot.commands.Shoot;
import frc.robot.commands.autos.AutoDependencies;
import frc.team4201.lib.command.Auto;
public class ShootNearStart extends Auto {
    public ShootNearStart(AutoDependencies deps, BooleanSupplier flipToRight) {
      try {
        var swerveDrive = deps.swerveDrive;
        var vision = deps.vision;
        var flywheel = deps.flywheel;
        var hood = deps.hood;
        var indexer = deps.indexer;
        var uptake = deps.uptake;
        var intake = deps.intake;
        
        var stopRequest = new SwerveRequest.ApplyRobotSpeeds();
        var traj = swerveDrive.getTrajectoryUtils();

        var path
            = PathPlannerPath.fromPathFile("ShootNearStart");
        
        addCommands(
            Commands.runOnce(() -> deps.flywheel.setRPMOutputFOC(Shoot.getShotForDistance(Meters.of(1.45650895207174)).shooterRPM)), // I literally just copied this from IntakeFromNeutral this is so bad omg
            getPathCommand(traj, path, flipToRight).andThen(() -> swerveDrive.setControl(stopRequest)),
            new ParallelCommandGroup(
                new Shoot(flywheel, hood, vision, swerveDrive),
                new WaitCommand(2).andThen(new Fire(intake, indexer, uptake)) // TODO: Auto-fire once auto align, shooter hood, and flywheel RPM are ready
            ).withTimeout(4) // TODO: Also auto-detect when we're done shooting
        );
      } catch (Exception e) {
        DriverStation.reportError("Failed to load path for ShootNearStart", e.getStackTrace());
        addCommands(new InstantCommand());
      }
    }
}
