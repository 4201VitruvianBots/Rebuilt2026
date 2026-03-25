// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Vision;

/**
 * Stores all subsystem dependencies needed by autonomous routines. This avoids passing long
 * parameter lists to each auto constructor.
 */
public class AutoDependencies {
  public final CommandSwerveDrivetrain swerveDrive;
  public final Intake intake;
  public final Vision vision;
  public final Flywheel flywheel;
  public final Hood hood;
  public final IntakePivot intakePivot;
  public final Indexer indexer;
  public final Uptake uptake;

  public AutoDependencies(
      CommandSwerveDrivetrain swerveDrive,
      Intake intake,
      Vision vision,
      Flywheel flywheel,
      Hood hood,
      IntakePivot intakePivot,
      Indexer indexer,
      Uptake uptake) {
    this.swerveDrive = swerveDrive;
    this.intake = intake;
    this.vision = vision;
    this.flywheel = flywheel;
    this.hood = hood;
    this.intakePivot = intakePivot;
    this.indexer = indexer;
    this.uptake = uptake;
  }
}
