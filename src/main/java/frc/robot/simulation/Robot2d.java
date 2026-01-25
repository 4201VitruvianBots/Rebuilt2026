// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.derive;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Robot2d extends SubsystemBase {
  private static final double pixelsPerInch = 631 / 26.5; // Non-curved edge of the drivebase in the image is 26.5 inches long and 631 pixels long
  private static final DistanceUnit Pixels = derive(Units.Inches).splitInto(pixelsPerInch).named("Pixels").symbol("px").make();
  
  // TODO: For some reason, the line width value needs to be 12 times larger. Fix
  private static final DistanceUnit LineWidthInches = derive(Units.Inches).splitInto(12).named("LineWidthInches").symbol("lw in").make();
  
  private final Distance robotCanvasX = Pixels.of(1170);
  private final Distance robotCanvasY = Pixels.of(818);
  
  private final Mechanism2d m_robot = new Mechanism2d(robotCanvasX.in(Inches), robotCanvasY.in(Inches));
  
  /** Map of subsystems for Robot2d to update */
  private final Map<String, Subsystem> m_subsystemMap = new HashMap<>();
  
  // Includes bumper width
  private final MechanismRoot2d m_chassisRoot =
    m_robot.getRoot("chassisRoot", Pixels.of(220).in(Inches), Pixels.of(156).in(Inches));
  private final MechanismLigament2d m_chassis = 
    new MechanismLigament2d("chassis", Inches.of(30.5).in(Inches), 0);
    
  /** Creates a new Robot2d. */
  public Robot2d() {
    m_chassis.setLineWeight(Inches.of(4.5).in(LineWidthInches));
    m_chassisRoot.append(m_chassis);
    
    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Robot2d", m_robot);
    }
  }
  
  public void registerSubsystems(Subsystem... subsystems) {
    for (Subsystem subsystem : subsystems) {
      if (subsystem != null) {
        m_subsystemMap.put(subsystem.getName(), subsystem);
      } else {
        DriverStation.reportWarning("[Robot2d] Attempting to register null subsystem!", true);
      }
    }
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
