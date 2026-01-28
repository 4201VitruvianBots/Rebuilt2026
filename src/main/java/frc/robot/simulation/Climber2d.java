// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.SIM.LineWidthInches;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.team4201.lib.simulation.visualization.Elevator2d;
import frc.team4201.lib.simulation.visualization.configs.Elevator2dConfig;

/** This simulation is so complex I decided we had to make it a separate class. */
public class Climber2d {
  // Climber belt. Mainly used as a visual reference for the max height of the climber
  
  // Climber carriage? Is that the correct term? Idk im a programmer it makes sense to me
  private final Elevator2d m_climber = 
    new Elevator2d(new Elevator2dConfig("Climber",
      new Color8Bit(255, 0, 255), // Magenta color for climber
      Inches.of(6.0)
    ).withLineWidth(Inches.of(1.675).in(LineWidthInches))
    .withSuperStructureOffset(Inches.of(0)) // Idk why I needed to add this
    .withAngleOffset(Degrees.of(90)) // Straight up
  );
  
  // Top hook
  
  // Bottom hook
}
