// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.AutoShoot;
import frc.team4201.lib.command.Auto;

public class SimboticsAuto extends Auto {
  public SimboticsAuto(AutoDependencies deps) {
    try {
      var swerveDrive = deps.swerveDrive;

      // Only for testing purposes, this auto is dysfunctional
      var m_path1 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("1678pt1");
      var m_path2 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("1678pt2");

      addCommands(
          m_path1,
          new AutoShoot(deps, 2.7),
          m_path2, 
          new AutoShoot(deps, 2.7));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for Simbotics Auto", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
