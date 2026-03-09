// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.routines;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.AutoShoot;
import frc.robot.constants.INTAKE;
import frc.robot.constants.UPTAKE.UPTAKE_SPEED_RPM;
import frc.robot.subsystems.Intake;

public class CenterDepot extends SequentialCommandGroup {
  public CenterDepot(AutoDependencies deps) {
    try {
      var swerveDrive = deps.swerveDrive;
      var vision = deps.vision;
      var flywheel = deps.flywheel;
      var hood = deps.hood;
      var intake = deps.intake;
      var intakePivot = deps.intakePivot;
      var indexer = deps.indexer;
      var uptake = deps.uptake;

      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var m_path1 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("CenterDepot1");
      var m_path2 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("CenterDepot2");
      var m_path3 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("CenterDepot3");
      addCommands(
          m_path1.andThen(() -> swerveDrive.setControl(stopRequest)),
          new AutoShoot(deps, 2),
          new ParallelDeadlineGroup(
              m_path2.andThen(() -> swerveDrive.setControl(stopRequest)),
              new IntakeCommand(intake, intakePivot, uptake)
          ),
          m_path3.andThen(() -> swerveDrive.setControl(stopRequest)),
          new AutoShoot(deps, 3)
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for CenterDepot", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
