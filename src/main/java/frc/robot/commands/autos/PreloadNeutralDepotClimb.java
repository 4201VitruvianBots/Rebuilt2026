// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Vision;
import frc.team4201.lib.command.Auto;

public class PreloadNeutralDepotClimb extends Auto {
  public PreloadNeutralDepotClimb(
      CommandSwerveDrivetrain swerveDrive,
      Intake intake,
      Vision vision,
      Flywheel flywheel,
      Hood hood,
      IntakePivot intakePivot,
      Indexer indexer,
      Uptake uptake) {
    try {
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var m_path1 =

swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb1");
      var m_path2 =

swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb2");
      var m_path3 =

swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb3");
      var m_path4 =

swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb4");
      var m_path5 =

swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb5");
      var m_path6 =

swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb6");
      var m_path7 =

swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb7");

      addCommands(
          m_path1.andThen(() -> swerveDrive.setControl(stopRequest)),
          new Shoot(flywheel, vision, hood).withTimeout(3),
          m_path2.andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelRaceGroup(
              new IntakeCommand(intake, intakePivot, indexer, uptake),
              m_path3.andThen(() -> swerveDrive.setControl(stopRequest))),
          m_path4.andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelCommandGroup(
              new Shoot(flywheel, vision, hood).withTimeout(3),
              m_path5.andThen(() -> swerveDrive.setControl(stopRequest))),
          new ParallelRaceGroup(
              new IntakeCommand(intake, intakePivot, indexer, uptake),
              m_path6.andThen(() -> swerveDrive.setControl(stopRequest))),
          m_path7.andThen(() -> swerveDrive.setControl(stopRequest))
          // Todo: add climb (command not yet implemented in this branch)
          );
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load path for PreloadNeutralDepotClimb", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
