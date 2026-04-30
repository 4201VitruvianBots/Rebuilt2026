// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.segments;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.PrepareFlywheel;
import frc.robot.constants.INTAKE.ROLLERS.INTAKE_STATE;
import frc.robot.constants.ROBOT.TWO_CYCLE_PATH;
import frc.team4201.lib.command.Auto;

public class IntakeFromNeutralNoCross extends Auto {

  public static void registerNamedCommands(AutoDependencies deps) {
    NamedCommands.registerCommand(
        "prepareFlywheelForNearHub",
        new PrepareFlywheel(deps.flywheel, Meters.of(1.45650895207174))
            .andThen(new PrintCommand("[AUTO] Preparing flywheel for near hub shot")));
  }

  public IntakeFromNeutralNoCross(
      AutoDependencies deps, BooleanSupplier flipToRight, TWO_CYCLE_PATH selectedPath) {
    try {
      var swerveDrive = deps.swerveDrive;
      var vision = deps.vision;
      var intake = deps.intake;
      var intakePivot = deps.intakePivot;
      var uptake = deps.uptake;

      PathPlannerPath path = PathPlannerPath.fromPathFile(selectedPath.getPathName());

      addCommands(
          new ParallelDeadlineGroup(
                  getPathCommand(swerveDrive, path, flipToRight),
                  new IntakeCommand(intake, intakePivot, uptake),
                  new PrintCommand("[AUTO] Crossing over bump and intaking..."))
              .andThen(new PrintCommand("[AUTO] Finished crossing over bump"))
              );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for IntakeFromNeutral", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
