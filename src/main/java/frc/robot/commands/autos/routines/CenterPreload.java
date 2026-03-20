// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Shoot;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.constants.UPTAKE.UPTAKE_SPEED;
import frc.team4201.lib.command.Auto;

public class CenterPreload extends Auto {
  public CenterPreload(AutoDependencies deps) {
    try {
      var swerveDrive = deps.swerveDrive;
      var vision = deps.vision;
      var flywheel = deps.flywheel;
      var hood = deps.hood;
      var uptake = deps.uptake;

      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var m_path1 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("CenterPreload1");
      var m_path2 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("CenterPreload2");

      addCommands(
          m_path1.andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelCommandGroup(
                  new Shoot(flywheel, hood, vision, swerveDrive),
                  uptake.percentCommand(UPTAKE_SPEED.SHOOTING.get()))
              .withTimeout(3));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for CenterPreload", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
