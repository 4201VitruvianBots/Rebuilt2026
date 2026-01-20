// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SHOOTERMOTORS.ShooterRPS;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterRollers;

public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ShooterRollers m_shooterRollers;

  private final ShooterHood m_shooterHood;

  private final ShooterRPS m_rps;
  private final Angle m_angle;

  public Shoot(
      ShooterRollers shooterRollers, ShooterHood shooterHood, ShooterRPS rps, Angle angle) {
    m_shooterRollers = shooterRollers;
    m_rps = rps;
    m_shooterHood = shooterHood;
    m_angle = angle;

    addRequirements(shooterRollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterRollers.setRPSOutputFOC(m_rps.getRPS());
    m_shooterHood.setAngle(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterRollers.setRPSOutputFOC(ShooterRPS.IDLE.getRPS());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
