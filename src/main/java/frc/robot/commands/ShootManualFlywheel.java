// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SHOOTERMOTORS.ManualRPM;
import frc.robot.subsystems.ShooterRollers;

public class ShootManualFlywheel extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ShooterRollers m_shooterRollers;

  public ShootManualFlywheel(ShooterRollers shooterRollers) {
    m_shooterRollers = shooterRollers;

    addRequirements(shooterRollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double TargetRPM = m_shooterRollers.m_rpmSubscriber.get() / 3;
    m_shooterRollers.setManualRPMOutputFOC(TargetRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterRollers.setTorqueCurrentOutputFOC(Volts.of(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
