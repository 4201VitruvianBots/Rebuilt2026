// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.SIM.LineWidthInches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.CLIMBER;
import frc.team4201.lib.simulation.visualization.Arm2d;
import frc.team4201.lib.simulation.visualization.Elevator2d;
import frc.team4201.lib.simulation.visualization.configs.Arm2dConfig;
import frc.team4201.lib.simulation.visualization.configs.Elevator2dConfig;

/** This simulation is so complex I decided we had to make it a separate class. */
public class Climber2d {
  private Color8Bit bottomClimberColor = new Color8Bit(219, 0, 255); // Blue-ish magenta color for bottom climber
  
  private Color8Bit topClimberColor = new Color8Bit(255, 0, 255); // Pure magenta color for top climber
  
  // Climber bottom carriage? Is that the correct term? Idk im a programmer it makes sense to me
  private final Elevator2d m_climberBottomCarriage;
  
  // Climber top carriage? Is that the correct term? Idk im a programmer it makes sense to me
  private final Elevator2d m_climberTopCarriage;
  
  // Bottom hook
  private final Arm2d m_climberBottomHook;
  
  // Top hook
  private final Arm2d m_climberTopHook;
  
  public Climber2d(MechanismRoot2d parentRoot) {
    m_climberBottomCarriage = 
        new Elevator2d(new Elevator2dConfig("Climber_bottomCarriage",
        new Color8Bit(219, 0, 255), 
        CLIMBER.upperLimit // Bottom hook starts at the upper limit of the climber
        ).withLineWidth(Inches.of(1.675).in(LineWidthInches))
        .withSuperStructureOffset(Inches.of(0)) // Idk why I needed to add this
        .withAngleOffset(Degrees.of(90)), // Straight up
        parentRoot);
    m_climberBottomHook =
        new Arm2d(new Arm2dConfig("Climber_bottomHook",
        bottomClimberColor,
        Degrees.of(-180), // Pointing straight down relative to the carriage
        Inches.of(5.81) // Length of the bottom hook
        ).withLineWidth(Inches.of(1.675).in(LineWidthInches)), m_climberBottomCarriage.getLastStageLigament());
    m_climberTopCarriage = 
        new Elevator2d(new Elevator2dConfig("Climber_topCarriage",
        new Color8Bit(255, 0, 255), 
        Inches.of(6.0) // Top hook starts 6 inches above the bottom of the climber
        ).withLineWidth(Inches.of(1.675).in(LineWidthInches))
        .withSuperStructureOffset(Inches.of(0)) // Idk why I needed to add this
        .withAngleOffset(Degrees.of(90)) // Straight up
        );
    m_climberTopHook =
        new Arm2d(new Arm2dConfig("Climber_topHook",
        topClimberColor,
        Degrees.of(-180), // Pointing straight down relative to the carriage
        Inches.of(5.81) // Length of the bottom hook
        ).withLineWidth(Inches.of(1.675).in(LineWidthInches)), m_climberTopCarriage.getLastStageLigament());
  }
  
  public void update(Distance height, LinearVelocity velocity) {
    m_climberTopCarriage.update(height.plus(Inches.of(6.0)), velocity);
    m_climberBottomCarriage.update(CLIMBER.upperLimit.minus(height), velocity);
    
    // When the top carriage starts at the bottom, the top hook stays at -180 degrees. It slowly moves to -90 degrees as it goes up, reaching -90 degrees at 1.25 inches.
    double topHookAngle = -180.0;
    if (height.in(Inches) >= 1.25) {
      topHookAngle = -90.0;
    } else {
      topHookAngle = -180.0 + (90.0 * (height.in(Inches) / 1.25));
    }
    m_climberTopHook.setAngle(Degrees.of(topHookAngle));
    
    // When the bottom carriage starts at the top, the bottom hook stays at -180 degrees. It slowly moves to -135 degrees as it goes down, reaching -135 degrees at 1.25 inches below max height.
    double bottomHookAngle = -180.0;
    Distance bottomHookTriggerHeight = CLIMBER.upperLimit.minus(Inches.of(1.25));
    if (height.in(Inches) <= bottomHookTriggerHeight.in(Inches)) {
      bottomHookAngle = -135.0;
    } else {
      bottomHookAngle = -180.0 + (45.0 * ((CLIMBER.upperLimit.minus(height)).in(Inches) / 1.25));
    }
    m_climberBottomHook.setAngle(Degrees.of(bottomHookAngle));
  }
}
