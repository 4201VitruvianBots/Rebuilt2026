// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.segments;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Fire;
import frc.robot.commands.Shoot;
import frc.robot.commands.autos.AutoDependencies;
import frc.team4201.lib.command.Auto;
import java.util.function.BooleanSupplier;

public class ShootNearStart extends Auto {
  public ShootNearStart(AutoDependencies deps, BooleanSupplier flipToRight) {
    try {
      var swerveDrive = deps.swerveDrive;
      var vision = deps.vision;
      var flywheel = deps.flywheel;
      var hood = deps.hood;
      var indexer = deps.indexer;
      var uptake = deps.uptake;
      var intake = deps.intake;

      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var path = PathPlannerPath.fromPathFile("ShootNearStart");

      addCommands(
          Commands.runOnce(
                  () ->
                      deps.flywheel.setRPMOutput(
                          Shoot.getShotForDistance(Meters.of(1.45650895207174)).shooterRPM))
              .andThen(
                  new PrintCommand(
                      "[AUTO] Preparing flywheel for near hub shot")), // I literally just copied
          // this from
          // IntakeFromNeutral this is
          // so bad omg
          getPathCommand(swerveDrive, path, flipToRight)
              .andThen(() -> swerveDrive.setControl(stopRequest))
              .andThen(new PrintCommand("[AUTO] Finished moving to shooting position")),
          new ParallelCommandGroup(
                  new Shoot(flywheel, hood, vision, swerveDrive),
                  new WaitCommand(1)
                      .andThen(
                          new Fire(
                              intake, indexer,
                              uptake, swerveDrive)) // TODO: Auto-fire once auto align, shooter hood, and
                  // flywheel RPM are ready
                  )
              .withTimeout(2)
              .andThen(
                  new PrintCommand(
                      "[AUTO] Finished shooting from near start")) // TODO: Also auto-detect when
          // we're done shooting
          );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for ShootNearStart", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
