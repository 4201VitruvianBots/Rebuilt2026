// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.Climb;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Shoot;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.UPTAKE.UPTAKE_SPEED;
import frc.robot.subsystems.*;
import frc.team4201.lib.command.Auto;

public class SideNeutralDepotClimb extends Auto {
  public SideNeutralDepotClimb(
      CommandSwerveDrivetrain swerveDrive,
      Intake intake,
      Vision vision,
      Flywheel flywheel,
      Hood hood,
      IntakePivot intakePivot,
      Indexer indexer,
      Uptake uptake,
      Climber climber) {
    try {
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var m_path1 =
          swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("SideNeutralDepotClimb1");
      var m_path2 =
          swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("SideNeutralDepotClimb2");
      var m_path3 =
          swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("SideNeutralDepotClimb3");
      var m_path4 =
          swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("SideNeutralDepotClimb4");
      var m_path5 =
          swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("SideNeutralDepotClimb5");
      var m_path6 =
          swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("SideNeutralDepotClimb6");

      addCommands(
          m_path1.andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelCommandGroup(
                  new Shoot(flywheel, hood, vision, swerveDrive),
                  uptake.command(UPTAKE_SPEED.UPTAKING))
              .withTimeout(3),
          m_path2.andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelRaceGroup(
              new IntakeCommand(intake, intakePivot, indexer, uptake),
              m_path3.andThen(() -> swerveDrive.setControl(stopRequest))),
          m_path4.andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelRaceGroup(
              new IntakeCommand(intake, intakePivot, indexer, uptake),
              m_path5.andThen(() -> swerveDrive.setControl(stopRequest))),
          new ParallelCommandGroup(
              new Shoot(flywheel, hood, vision, swerveDrive).withTimeout(3),
              new Climb(climber, CLIMBER.CLIMBER_SETPOINT.LEVEL_ONE)
                  .until(
                      () ->
                          Math.abs(
                                  climber.getHeight().in(Meter)
                                      - CLIMBER.CLIMBER_SETPOINT.LEVEL_ONE.getSetpoint().in(Meter))
                              < 0.1)),
          m_path6.andThen(() -> swerveDrive.setControl(stopRequest)),
          new Climb(climber, CLIMBER.CLIMBER_SETPOINT.START_POSITION));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for SideNeutralDepotClimb", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
