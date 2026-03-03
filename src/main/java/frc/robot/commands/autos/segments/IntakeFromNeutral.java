// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.segments;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.constants.INTAKE.ROLLERS;
import frc.team4201.lib.command.Auto;

public class IntakeFromNeutral extends Auto {
    private static boolean lostCenterRace = false;
    private static double averageCurrentBefore;
    private static double averageCurrentAfter;
    
    public static void registerNamedCommands(AutoDependencies deps) {
        NamedCommands.registerCommand("prepareFlywheelForNearHub", Commands.runOnce(() -> deps.flywheel.setRPMOutputFOC(Shoot.getShotForDistance(Meters.of(1.45650895207174)).shooterRPM))); // YAY magic numbers
        NamedCommands.registerCommand("measureIntakeBefore", Commands.startEnd(() -> {
           deps.intake.startCurrentFilter();
        }, () -> {
            averageCurrentBefore = deps.intake.endCurrentFilter(); // Note: UH OH this might start a race condition with the other current measurement command causing things to get reset improperly please check
        }));
        NamedCommands.registerCommand("measureIntakeAfter", Commands.startEnd(() -> {
            deps.intake.startCurrentFilter();
        }, () -> {
            averageCurrentAfter = deps.intake.endCurrentFilter(); // I was so tired when I wrote all this code lowkey
            lostCenterRace = (averageCurrentAfter - averageCurrentBefore) > ROLLERS.currentDifferenceThreshold;
        }));
    }
    
    public IntakeFromNeutral(AutoDependencies deps, boolean startingFromShoot, BooleanSupplier flipToRight) {
      try {
        var swerveDrive = deps.swerveDrive;
        var intake = deps.intake;
        var intakePivot = deps.intakePivot;
        var indexer = deps.indexer;
        var uptake = deps.uptake;
        
        var stopRequest = new SwerveRequest.ApplyRobotSpeeds();
        var traj = swerveDrive.getTrajectoryUtils();
        
        var crossOverBump =
            PathPlannerPath.fromPathFile(startingFromShoot ? "CrossOverBumpFromShooting" : "CrossOverBumpFromStart");
              
        var intakeFromCenter
            = PathPlannerPath.fromPathFile("IntakeFromCenter");
        var intakeFromSide
            = PathPlannerPath.fromPathFile("IntakeFromSide");
        
        var returnToAllianceZone
            = PathPlannerPath.fromPathFile("ReturnToAllianceZone");
        
        addCommands(
            new ParallelRaceGroup(
                getPathCommand(traj, crossOverBump, flipToRight),
                new IntakeCommand(intake, intakePivot, indexer, uptake)
            ).andThen(() -> swerveDrive.setControl(stopRequest)),
            new ParallelRaceGroup(
                getPathCommand(traj, lostCenterRace ? intakeFromSide : intakeFromCenter, flipToRight),
                new IntakeCommand(intake, intakePivot, indexer, uptake)
            ).andThen(() -> swerveDrive.setControl(stopRequest)),
            getPathCommand(traj, returnToAllianceZone, flipToRight)
                .andThen(() -> swerveDrive.setControl(stopRequest))
        );
      } catch (Exception e) {
        DriverStation.reportError("Failed to load path for IntakeFromNeutral", e.getStackTrace());
        addCommands(new InstantCommand());
      }
    }
}
