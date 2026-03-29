// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.AutoShootManual;
import frc.team4201.lib.command.Auto;

public class CenterPreload extends Auto {
  public CenterPreload(AutoDependencies deps) {
    try {
      var swerveDrive = deps.swerveDrive;

      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      PathPlannerPath m_path1 = PathPlannerPath.fromPathFile("CenterPreload1");

      var m_command1 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(m_path1);
      var m_command2 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("CenterPreload2");

      setInitialPath(m_path1);

      addCommands(
          m_command1.andThen(() -> swerveDrive.setControl(stopRequest)),
          new AutoShootManual(deps, 3.0),
          m_command2.andThen(() -> swerveDrive.setControl(stopRequest)));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for CenterPreload", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
