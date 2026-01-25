// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SHOOTER.FLYWHEEL.SHOOTER_VELOCITY;
import frc.robot.subsystems.ShooterFlywheel;

public class ShootFlywheel extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ShooterFlywheel m_shooterFlywheel;

  private final SHOOTER_VELOCITY m_rpm;

  public ShootFlywheel(ShooterFlywheel shooterFlywheel, SHOOTER_VELOCITY rpm) {
    m_shooterFlywheel = shooterFlywheel;
    m_rpm = rpm;

    addRequirements(shooterFlywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterFlywheel.setRPMOutputFOC(m_rpm.getRPM());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterFlywheel.setVoltageOutputFOC(Volts.of(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
