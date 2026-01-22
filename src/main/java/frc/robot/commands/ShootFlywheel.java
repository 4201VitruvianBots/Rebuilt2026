// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SHOOTERMOTORS.ShooterVelocity;
import frc.robot.subsystems.ShooterRollers;

public class ShootFlywheel extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ShooterRollers m_shooterRollers;

  private final ShooterVelocity m_rpm;

  public ShootFlywheel(ShooterRollers shooterRollers, ShooterVelocity rpm) {
    m_shooterRollers = shooterRollers;
    m_rpm = rpm;

    addRequirements(shooterRollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterRollers.setRPMOutputFOC(m_rpm.getRPM());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterRollers.setVoltageOutputFOC(Volts.of(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
