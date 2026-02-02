// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterRollers;
import frc.robot.subsystems.Vision;
import frc.team4201.lib.command.Auto;

public class PreloadCenter extends Auto {
  public PreloadCenter(
          CommandSwerveDrivetrain swerveDrive,
          Intake intake,
          Vision vision,
          ShooterRollers shooterRollers) {
    try {
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var m_path1 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadCenter1");

      addCommands(
        m_path1.andThen(() -> swerveDrive.setControl(stopRequest)),
        new Shoot(swerveDrive, shooterRollers, vision).withTimeout(4)
      );
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load path for PreloadCenter", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
