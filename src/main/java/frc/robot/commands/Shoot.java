// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SHOOTER.FLYWHEEL.SHOOTER_VELOCITY;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterFlywheel;

public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ShooterFlywheel m_shooterFlywheel;

  private final ShooterHood m_shooterHood;

  private final SHOOTER_VELOCITY m_rpm;
  private final Angle m_angle;

  public Shoot(
      ShooterFlywheel shooterFlywheel, ShooterHood shooterHood, SHOOTER_VELOCITY rpm, Angle angle) {
    m_shooterFlywheel = shooterFlywheel;
    m_rpm = rpm;
    m_shooterHood = shooterHood;
    m_angle = angle;

    addRequirements(shooterFlywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterFlywheel.setRPMOutputFOC(m_rpm.getRPM());
    m_shooterHood.setAngle(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterFlywheel.setRPMOutputFOC(SHOOTER_VELOCITY.IDLE.getRPM());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
