// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.segments;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.AutoShoot;
import frc.robot.commands.autos.PrepareFlywheel;
import frc.team4201.lib.command.Auto;
import java.util.function.BooleanSupplier;

public class ShootNearStart extends Auto {
  public ShootNearStart(AutoDependencies deps, BooleanSupplier flipToRight) {
    try {
      var swerveDrive = deps.swerveDrive;
      var flywheel = deps.flywheel;

      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var path = PathPlannerPath.fromPathFile("1678pt1");

      addCommands(
          new PrepareFlywheel(flywheel, Meters.of(1.45650895207174))
              .andThen(new PrintCommand("[AUTO] Preparing flywheel for near hub shot")),
          new AutoShoot(deps, 3.0) // TODO: Tune this timeout
              .andThen(
                  new PrintCommand(
                      "[AUTO] Finished shooting from near start"))
          );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for ShootNearStart", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
