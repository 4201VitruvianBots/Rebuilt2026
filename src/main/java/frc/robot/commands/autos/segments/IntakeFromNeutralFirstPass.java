// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.segments;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.PrepareFlywheel;
import frc.team4201.lib.command.Auto;
import java.util.function.BooleanSupplier;

public class IntakeFromNeutralFirstPass extends Auto {

  public static void registerNamedCommands(AutoDependencies deps) {
    NamedCommands.registerCommand(
        "prepareFlywheelForNearHub",
        new PrepareFlywheel(deps.flywheel, Meters.of(1.45650895207174))
            .andThen(new PrintCommand("[AUTO] Preparing flywheel for near hub shot")));
  }

  public IntakeFromNeutralFirstPass(
      AutoDependencies deps, boolean startingFromShoot, BooleanSupplier flipToRight) {
    try {
      var swerveDrive = deps.swerveDrive;
      var intake = deps.intake;
      var intakePivot = deps.intakePivot;
      var uptake = deps.uptake;

      var path =
          PathPlannerPath.fromPathFile("1678pt1");

      addCommands(
          new ParallelDeadlineGroup(
                  getPathCommand(swerveDrive, path, flipToRight),
                  new IntakeCommand(intake, intakePivot, uptake),
                  new PrintCommand("[AUTO] Crossing over bump and intaking..."))
              .andThen(new PrintCommand("[AUTO] Finished crossing over bump")));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for IntakeFromNeutral", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
