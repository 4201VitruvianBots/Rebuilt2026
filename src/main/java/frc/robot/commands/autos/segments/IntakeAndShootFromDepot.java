// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.segments;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.team4201.lib.command.Auto;

public class IntakeAndShootFromDepot extends Auto {
    public IntakeAndShootFromDepot(AutoDependencies deps) {
    try {
        var swerveDrive = deps.swerveDrive;
        var climber = deps.climber;
        var intake = deps.intake;
        var vision = deps.vision;
        var flywheel = deps.flywheel;
        var hood = deps.hood;
        var intakePivot = deps.intakePivot;
        var indexer = deps.indexer;
        var uptake = deps.uptake;
        
        var stopRequest = new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds();

        var intakeFromDepot
            = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("IntakeFromDepot");
        var shootFromDepot
            = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("ShootFromDepot");
        
        addCommands(
            
        );
      } catch (Exception e) {
        DriverStation.reportError("Failed to load path for IntakeAndShootFromDepot", e.getStackTrace());
        addCommands(new InstantCommand());
      }
    }
}
