// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Shoot;
import frc.robot.constants.UPTAKE.UPTAKE_SPEED;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Vision;
import frc.team4201.lib.command.Auto;
import frc.team4201.lib.utils.TrajectoryUtils;

import java.util.function.BooleanSupplier;

public class SideNeutralTwice extends Auto {
  public SideNeutralTwice(
      CommandSwerveDrivetrain swerveDrive,
      Intake intake,
      Vision vision,
      Flywheel flywheel,
      Hood hood,
      IntakePivot intakePivot,
      Indexer indexer,
      Uptake uptake,
      BooleanSupplier flipPath,
      boolean rushCenter) {
    try {
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      TrajectoryUtils trajectoryUtils = swerveDrive.getTrajectoryUtils();

      var m_path1 = PathPlannerPath.fromPathFile("SideNeutralTwice1");
      var m_path2 = PathPlannerPath.fromPathFile("SideNeutralTwice2");
      var m_path3 = PathPlannerPath.fromPathFile("SideNeutralTwice3");
      var m_path4 = PathPlannerPath.fromPathFile("SideNeutralTwice4");
      var m_path5 = PathPlannerPath.fromPathFile("SideNeutralTwice5");
      var m_path6 = PathPlannerPath.fromPathFile("SideNeutralTwice6");
      var m_path7 = PathPlannerPath.fromPathFile("SideNeutralTwice7");

      Timer timer = new Timer();

      addCommands(
          new InstantCommand(timer::restart),
          getPathCommand(trajectoryUtils, m_path1, flipPath)
              .andThen(() -> swerveDrive.setControl(stopRequest)),
          rushCenter
              ? new InstantCommand()
              : new ParallelCommandGroup(
                      new Shoot(flywheel, hood, vision, swerveDrive),
                      uptake.command(UPTAKE_SPEED.UPTAKING))
                  .withTimeout(1), // Don't shoot preload if we're trying to rush the center
          getPathCommand(trajectoryUtils, m_path2, flipPath)
              .andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelRaceGroup(
              new IntakeCommand(intake, intakePivot, indexer, uptake),
              getPathCommand(trajectoryUtils, m_path3, flipPath)
                  .andThen(() -> swerveDrive.setControl(stopRequest))),
          getPathCommand(trajectoryUtils, m_path4, flipPath)
              .andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelCommandGroup(
                  new Shoot(flywheel, hood, vision, swerveDrive),
                  uptake.command(UPTAKE_SPEED.UPTAKING))
              .withTimeout(2),
          // This code just repeats the last four steps again.
          getPathCommand(trajectoryUtils, m_path5, flipPath)
              .andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelRaceGroup(
              new IntakeCommand(intake, intakePivot, indexer, uptake),
              getPathCommand(trajectoryUtils, m_path6, flipPath)
                  .andThen(() -> swerveDrive.setControl(stopRequest))),
          getPathCommand(trajectoryUtils, m_path7, flipPath)
              .andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelCommandGroup(
                  new Shoot(flywheel, hood, vision, swerveDrive),
                  uptake.command(UPTAKE_SPEED.UPTAKING))
              .withTimeout(3),
          new InstantCommand(()->System.out.println(timer.get()))
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for SideNeutralTwice", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
