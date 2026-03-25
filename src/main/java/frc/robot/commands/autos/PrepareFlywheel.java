// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Flywheel;

// Prepare to shoot flywheel from a given distance from the target.
// This is used in auto to prepare the flywheel for shooting in advance of running the command.
public class PrepareFlywheel extends InstantCommand {
  private Flywheel m_flywheel;
  private Distance m_shootingDistance;

  public PrepareFlywheel(Flywheel flywheel, Distance shootingDistance) {
    m_flywheel = flywheel;
    m_shootingDistance = shootingDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_flywheel.setRPMOutput(Shoot.getShotForDistance(m_shootingDistance).shooterRPM);
  }
}
