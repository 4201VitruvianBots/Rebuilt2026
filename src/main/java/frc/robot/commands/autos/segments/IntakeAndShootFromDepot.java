// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.segments;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.AutoShoot;
import frc.robot.commands.autos.PrepareFlywheel;
import frc.team4201.lib.command.Auto;

public class IntakeAndShootFromDepot extends Auto {
  public static void registerNamedCommands(AutoDependencies deps) {
    NamedCommands.registerCommand(
        "prepareFlywheelForDepot", new PrepareFlywheel(deps.flywheel, Meters.of(3.31521515713456)));
  }

  public IntakeAndShootFromDepot(AutoDependencies deps) {
    try {
      var swerveDrive = deps.swerveDrive;
      var intake = deps.intake;
      var intakePivot = deps.intakePivot;
      var uptake = deps.uptake;

      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var intakeFromDepot =
          swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("IntakeFromDepot");
      var shootFromDepot =
          swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("ShootFromDepot");

      addCommands(
          new ParallelDeadlineGroup(
                  intakeFromDepot,
                  new IntakeCommand(intake, intakePivot, uptake),
                  new PrintCommand("[AUTO] Intaking from depot..."))
              .andThen(new PrintCommand("[AUTO] Finished intaking from depot")),
          shootFromDepot
              .andThen(new PrintCommand("[AUTO] Ready to shoot from depot")),
          new AutoShoot(deps, 3.0) // TODO: Tune this timeout
              .andThen(new PrintCommand("[AUTO] Finished shooting from depot")));
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load path for IntakeAndShootFromDepot", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
