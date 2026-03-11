// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.segments;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.PrepareFlywheel;
import frc.robot.constants.INTAKE.ROLLERS;
import frc.team4201.lib.command.Auto;
import java.util.function.BooleanSupplier;

public class IntakeFromNeutral extends Auto {
  private static boolean lostCenterRace = false;
  private static double averageCurrentBefore;
  private static double averageCurrentAfter;

  public static void registerNamedCommands(AutoDependencies deps) {
    // Check whether we've lost the race to intake to the center by measuring the current of the intake rollers before and after crossing the bump
    NamedCommands.registerCommand(
        "prepareFlywheelForNearHub",
        new PrepareFlywheel(deps.flywheel, Meters.of(1.45650895207174))
            .andThen(new PrintCommand("[AUTO] Preparing flywheel for near hub shot")));
    NamedCommands.registerCommand(
        "measureIntakeBefore",
        Commands.startEnd(
            () -> {
              deps.intake.startCurrentFilter();
              System.out.println("[AUTO] Measuring current before going over bump...");
            },
            () -> {
              System.out.println("Finished pre-bump current measurements");
              averageCurrentBefore =
                  deps.intake
                      .endCurrentFilter(); // TODO: UH OH this might start a race condition with the
              // other current measurement command causing things to
              // get reset improperly please check
            }));
    NamedCommands.registerCommand(
        "measureIntakeAfter",
        Commands.startEnd(
            () -> {
              deps.intake.startCurrentFilter();
              System.out.println("[AUTO] Measuring current after going over bump...");
            },
            () -> {
              averageCurrentAfter =
                  deps.intake
                      .endCurrentFilter(); // I was so tired when I wrote all this code lowkey
              lostCenterRace =
                  (averageCurrentAfter - averageCurrentBefore) > ROLLERS.currentDifferenceThreshold;
              System.out.println("[AUTO] Finished post-bump current measurements");
              System.out.println("[AUTO] Average current before: " + averageCurrentBefore);
              System.out.println("[AUTO] Average current after: " + averageCurrentAfter);
              System.out.println(
                  "[AUTO] Current difference: " + (averageCurrentAfter - averageCurrentBefore));
              System.out.println("[AUTO] Lost center? " + lostCenterRace);
            }));
  }

  public IntakeFromNeutral(
      AutoDependencies deps, boolean startingFromShoot, BooleanSupplier flipToRight) {
    try {
      var swerveDrive = deps.swerveDrive;
      var intake = deps.intake;
      var intakePivot = deps.intakePivot;
      var indexer = deps.indexer;
      var uptake = deps.uptake;

      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var crossOverBump =
          PathPlannerPath.fromPathFile(
              startingFromShoot ? "CrossOverBumpFromShooting" : "CrossOverBumpFromStart");

      var intakeFromCenter = PathPlannerPath.fromPathFile("IntakeFromCenter");
      var intakeFromSide = PathPlannerPath.fromPathFile("IntakeNearHub");

      var returnToAllianceZone = PathPlannerPath.fromPathFile("ReturnToAllianceZone");

      addCommands(
          new ParallelDeadlineGroup(
                  getPathCommand(swerveDrive, crossOverBump, flipToRight),
                  new IntakeCommand(intake, intakePivot, uptake),
                  new PrintCommand("[AUTO] Crossing over bump and intaking..."))
              .andThen(() -> swerveDrive.setControl(stopRequest))
              .andThen(new PrintCommand("[AUTO] Finished crossing over bump")),
          new ParallelDeadlineGroup(
                  getPathCommand(
                      swerveDrive, lostCenterRace ? intakeFromSide : intakeFromCenter, flipToRight),
                  new IntakeCommand(intake, intakePivot, uptake),
                  new PrintCommand(
                      "[AUTO] Intaking " + (lostCenterRace ? "near hub" : "from center") + "..."))
              .andThen(() -> swerveDrive.setControl(stopRequest))
              .andThen(
                  new PrintCommand(
                      "[AUTO] Finished intaking " + (lostCenterRace ? "near hub" : "from center"))),
          getPathCommand(swerveDrive, returnToAllianceZone, flipToRight)
              .andThen(() -> swerveDrive.setControl(stopRequest))
              .andThen(new PrintCommand("[AUTO] Returned to alliance zone")));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for IntakeFromNeutral", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
