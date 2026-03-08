// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Fire;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;

// Begins firing once the shooter is up to speed, and continues firing for a given duration. Used in
// auto routines to fire shots while running paths.
public class AutoShoot extends Command {
  // private final CommandSwerveDrivetrain m_swerveDrive; // TODO: check if drivetrain is at correct
  // heading for shooting as well before firing
  private final Flywheel m_flywheel;
  private final Hood m_hood;

  private Shoot m_shootCommand;
  private Fire m_fireCommand;
  private double m_fireDurationSeconds;

  /** Creates a new AutoShoot. */
  public AutoShoot(AutoDependencies deps, double fireDurationSeconds) {
    // m_swerveDrive = deps.swerveDrive;
    m_flywheel = deps.flywheel;
    m_hood = deps.hood;
    m_shootCommand = new Shoot(deps.flywheel, deps.hood, deps.vision, deps.swerveDrive);
    m_fireCommand = new Fire(deps.intake, deps.indexer, deps.uptake);
    m_fireDurationSeconds = fireDurationSeconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(m_shootCommand);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_flywheel.isAtRPMsetpoint()
        && m_hood.atSetpoint()
        && !CommandScheduler.getInstance().isScheduled(m_fireCommand)) {
      CommandScheduler.getInstance().schedule(m_fireCommand.withTimeout(m_fireDurationSeconds));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancel(m_shootCommand, m_fireCommand);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_fireCommand.isFinished();
  }
}
