// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.derive;

import java.util.HashMap;
import java.util.Map;

import javax.sound.sampled.Line;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4201.lib.simulation.visualization.*;
import frc.team4201.lib.simulation.visualization.configs.*;

public class Robot2d extends SubsystemBase {
  private static final double pixelsPerInch = 631 / 26.5; // Non-curved edge of the drivebase in the image is 26.5 inches long and 631 pixels long
  private static final DistanceUnit Pixels = derive(Units.Inches).splitInto(pixelsPerInch).named("Pixels").symbol("px").make();
  
  // For some reason, the line width value needs to be 12 times larger in order to actually match up with the visual thickness of lines in the Mechanism2d
  private static final DistanceUnit LineWidthInches = derive(Units.Inches).splitInto(12).named("LineWidthInches").symbol("lw in").make();
  
  private final Distance robotCanvasX = Pixels.of(1170);
  private final Distance robotCanvasY = Pixels.of(818);
  
  private final Mechanism2d m_robot = new Mechanism2d(robotCanvasX.in(Inches), robotCanvasY.in(Inches));
  
  private final Distance chassisRootX = Pixels.of(220);
  private final Distance chassisRootY = Pixels.of(156);
  
  // Includes bumper width
  private final MechanismRoot2d m_chassisRoot =
    m_robot.getRoot("chassisRoot", chassisRootX.in(Inches), chassisRootY.in(Inches));
  private final MechanismLigament2d m_chassis = 
    new MechanismLigament2d("chassis",
    Inches.of(30.5).in(Inches), // Full width including bumpers
 0);
  
  private final MechanismRoot2d m_intakeRoot =
    m_robot.getRoot("intakeRoot", 
     chassisRootX.plus(Inches.of(10.588)).in(Inches), // Distance from edge of bumper to center of intake pivot
     chassisRootY.plus(Inches.of(3.5)).in(Inches)); // Intake is ~3.5 inches above center of bumpers
  private final Arm2d m_intakePivot =
    new Arm2d(new Arm2dConfig(
        "intakePivot",
        new Color8Bit(255, 255, 255),
        Degrees.of(159), // Angle between chassis and first segment of intake when extended
        Inches.of(10.735) // Length of first segment of intake
    ).withLineWidth(Inches.of(1.465).in(LineWidthInches)), // Width of the intake pivot arm at its thinnest point
    m_intakeRoot);
    
  /** Map of subsystems for Robot2d to update */
  private final Map<String, Subsystem> m_subsystemMap = new HashMap<>();
  
  /** Creates a new Robot2d. */
  public Robot2d() {
    m_chassis.setLineWeight(Inches.of(4.5).in(LineWidthInches));
    m_chassisRoot.append(m_chassis);
    
    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Robot2d", m_robot);
      m_intakePivot.generateSubDisplay();
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
