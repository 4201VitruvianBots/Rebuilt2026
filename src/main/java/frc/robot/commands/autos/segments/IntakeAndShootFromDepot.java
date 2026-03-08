// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.segments;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Fire;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.autos.AutoDependencies;
import frc.team4201.lib.command.Auto;

public class IntakeAndShootFromDepot extends Auto {
  public static void registerNamedCommands(AutoDependencies deps) {
    NamedCommands.registerCommand(
        "prepareFlywheelForDepot",
        Commands.runOnce(
            () ->
                deps.flywheel.setRPMOutputFOC(
                    Shoot.getShotForDistance(Meters.of(3.31521515713456))
                        .shooterRPM))); // YAY more magic numbers
  }

  public IntakeAndShootFromDepot(AutoDependencies deps) {
    try {
      var swerveDrive = deps.swerveDrive;
      var intake = deps.intake;
      var vision = deps.vision;
      var flywheel = deps.flywheel;
      var hood = deps.hood;
      var intakePivot = deps.intakePivot;
      var indexer = deps.indexer;
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
              .andThen(() -> swerveDrive.setControl(stopRequest))
              .andThen(new PrintCommand("[AUTO] Finished intaking from depot")),
          shootFromDepot
              .andThen(() -> swerveDrive.setControl(stopRequest))
              .andThen(new PrintCommand("[AUTO] Ready to shoot from depot")),
          new ParallelCommandGroup(
                  new Shoot(flywheel, hood, vision, swerveDrive),
                  new WaitCommand(2)
                      .andThen(
                          new Fire(
                              intake, indexer,
                              uptake)) // TODO: Auto-fire once auto align, shooter hood, and
                  // flywheel RPM are ready
                  )
              .withTimeout(4)
              .andThen(
                  new PrintCommand(
                      "[AUTO] Finished shooting from depot")) // TODO: Also auto-detect when we're
          // done shooting
          );
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load path for IntakeAndShootFromDepot", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
