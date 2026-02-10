// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.derive;
import static frc.robot.Constants.SIM.LineWidthInches;

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
import frc.robot.Constants.FLYWHEEL;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Uptake;
import frc.team4201.lib.simulation.visualization.*;
import frc.team4201.lib.simulation.visualization.configs.*;
import java.util.HashMap;
import java.util.Map;

// Hopefully there isn't too much overhead from making this a subsystem.
public class Robot2d extends SubsystemBase {
  private static final double pixelsPerInch =
      631 / 26.5; // Non-curved edge of the drivebase in the image is 26.5 inches long and 631
  // pixels long
  private static final DistanceUnit Pixels =
      derive(Units.Inches).splitInto(pixelsPerInch).named("Pixels").symbol("px").make();

  private final Distance robotCanvasX = Pixels.of(1170);
  private final Distance robotCanvasY = Pixels.of(1170);

  private final Mechanism2d m_robot =
      new Mechanism2d(robotCanvasX.in(Meters), robotCanvasY.in(Meters));

  // Shared colors used by the robot visualization
  private final Color8Bit m_colorIntake = new Color8Bit(255, 192, 203); // Pink
  private final Color8Bit m_colorFlywheel = new Color8Bit(127, 127, 127); // Gray
  private final Color8Bit m_colorHood = new Color8Bit(0, 255, 255); // Aqua
  private final Color8Bit m_colorIndexer = new Color8Bit(255, 200, 0); // Yellow
  private final Color8Bit m_colorUptake = new Color8Bit(0, 255, 0); // Green

  private final Distance chassisRootX = Pixels.of(220);
  private final Distance chassisRootY = Pixels.of(121);

  // Includes bumper width
  private final MechanismRoot2d m_chassisRoot =
      m_robot.getRoot("chassisRoot", chassisRootX.in(Meters), chassisRootY.in(Meters));
  private final MechanismLigament2d m_chassis =
      m_chassisRoot.append(
          new MechanismLigament2d(
              "chassis",
              Inches.of(30.5).in(Meters), // Full width including bumpers
              0));

  // Intake
  private final MechanismRoot2d m_intakeRoot =
      m_robot.getRoot(
          "intakeRoot",
          chassisRootX
              .plus(Inches.of(10.588))
              .in(Meters), // Distance from edge of bumper to center of intake pivot
          chassisRootY
              .plus(Inches.of(3.5))
              .in(Meters)); // Intake is ~3.5 inches above center of bumpers
  private final Arm2d m_intakePivot =
      new Arm2d(
          new Arm2dConfig(
                  "Intake Pivot",
                  m_colorIntake,
                  Degrees.of(
                      159), // Angle between chassis and first segment of intake when extended
                  Inches.of(10.735) // Length of first segment of intake
                  )
              .withLineWidth(
                  Inches.of(1.465)
                      .in(LineWidthInches)), // Width of the intake pivot arm at its thinnest point
          m_intakeRoot);
  private final MechanismLigament2d m_intakeSegment1 =
      m_intakePivot
          .getLigament()
          .append(
              new MechanismLigament2d(
                  "Intake Segment 1",
                  Inches.of(2.0).in(Meters),
                  Degrees.of(21).in(Degrees),
                  Inches.of(1.25).in(LineWidthInches),
                  m_colorIntake));
  private final MechanismLigament2d m_intakeSegment2 =
      m_intakeSegment1.append(
          new MechanismLigament2d(
              "Intake Segment 2",
              Inches.of(8.27).in(Meters),
              Degrees.of(48.5).in(Degrees),
              Inches.of(2.679).in(LineWidthInches),
              m_colorIntake));

  // Indexer
  private final MechanismRoot2d m_indexerRoot =
      m_robot.getRoot(
          "indexerRoot",
          chassisRootX
              .plus(Inches.of(13.4))
              .in(Meters), // Distance from edge of bumper to furthest left indexer roller
          chassisRootY
              .plus(Inches.of(2.25))
              .in(Meters)); // Indexer is ~2.25 inches above center of bumpers
  private final MechanismLigament2d m_indexer =
      m_indexerRoot.append(
          new MechanismLigament2d(
              "Indexer",
              Inches.of(9.6).in(Meters), // Length of the indexer from first roller to last roller
              170)); // Angle of the indexer relative to the chassis

  // Uptake
  private final Distance uptakeRootX =
      chassisRootX.plus(
          Inches.of(18.725)); // Distance from edge of bumper to center of thinnest part of uptake
  private final Distance uptakeRootY =
      chassisRootY.plus(Inches.of(2.25)); // Indexer is ~2.25 inches above center of bumpers
  private final Distance uptakeWidth = Inches.of(6.65); // Width of the uptake at its thinnest point
  private final Distance uptakeHeight = Inches.of(18.5); // Height of the uptake
  private final MechanismRoot2d m_uptakeRoot =
      m_robot.getRoot("uptakeRoot", uptakeRootX.in(Meters), uptakeRootY.in(Meters));
  private final MechanismLigament2d m_uptake =
      m_uptakeRoot.append(
          new MechanismLigament2d(
              "Uptake",
              uptakeHeight.in(Meters), // Height of the uptake
              90)); // Straight up

  // Flywheel
  private final MechanismRoot2d m_flywheelRoot =
      m_robot.getRoot(
          "flywheelRoot",
          uptakeRootX
              .minus(uptakeWidth.div(2))
              .in(Meters), // Put the flywheel on the top left corner of the uptake
          uptakeRootY.plus(uptakeHeight).in(Meters));
  private final Flywheel2d m_flywheel =
      new Flywheel2d(
          new Flywheel2dConfig(
              "Flywheel",
              m_colorFlywheel, // Grey color for flywheel
              FLYWHEEL.radius // Radius of the flywheel
              ),
          m_flywheelRoot);

  // Shooter Hood
  private final Arm2d m_shooterHood =
      new Arm2d(
          new Arm2dConfig(
              "Shooter Hood",
              m_colorHood, // Aqua color for shooter hood
              Degrees.of(35), // TODO: Find actual angle
              Inches.of(10.0) // TODO: Find actual length
              ),
          m_flywheelRoot);

  // Climber
  private final MechanismRoot2d m_climberRoot =
      m_robot.getRoot(
          "climberRoot",
          uptakeRootX
              .plus(uptakeWidth)
              .plus(Inches.of(3.35 / 2))
              .in(Meters), // Put the flywheel on the top left corner of the uptake
          uptakeRootY.in(Meters));
  private final Climber2d m_climber = new Climber2d(m_climberRoot);

  // TODO: Add hopper, Vision, LEDs?

  /** Map of subsystems for Robot2d to update */
  private final Map<String, Subsystem> m_subsystemMap = new HashMap<>();

  /** Creates a new Robot2d. */
  public Robot2d() {
    m_chassis.setLineWeight(Inches.of(4.5).in(LineWidthInches)); // Bumpers are 4.5 inches thick
    m_indexer.setLineWeight(
        Inches.of(1.25).in(LineWidthInches)); // Diameter of the indexer rollers is 1.25 inches
    m_indexer.setColor(m_colorIndexer);
    m_uptake.setLineWeight(uptakeWidth.in(LineWidthInches));
    m_uptake.setColor(m_colorUptake);

    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Robot2d", m_robot);
      m_intakePivot.generateSubDisplay();
      m_flywheel.generateSubDisplay();
      m_shooterHood.generateSubDisplay();
      m_climber.generateSubDisplay();
      //   graphics = good
      //   glitches = None
      //   hackers = 0
      //   exploits = only the fun ones
      //   wasd = walk
      //   spacebar = jump
      //   helloWorld("print");
      // thank you joaquin
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
    if (m_subsystemMap.containsKey("Intake")) {
      var intakeSubsystem = (Intake) m_subsystemMap.get("Intake");
      VisualizationUtils.updateMotorColor(
          m_intakeSegment1, intakeSubsystem.getPercentOutput(), m_colorIntake);
      VisualizationUtils.updateMotorColor(
          m_intakeSegment2, intakeSubsystem.getPercentOutput(), m_colorIntake);
    }
    if (m_subsystemMap.containsKey("IntakePivot")) {
      var intakePivotSubsystem = (IntakePivot) m_subsystemMap.get("IntakePivot");
      m_intakePivot.update(intakePivotSubsystem.getAngle());
    }
    if (m_subsystemMap.containsKey("Indexer")) {
      var indexerSubsystem = (Indexer) m_subsystemMap.get("Indexer");
      VisualizationUtils.updateMotorColor(
          m_indexer, indexerSubsystem.getPercentOutput(), m_colorIndexer);
    }
    if (m_subsystemMap.containsKey("Uptake")) {
      var uptakeSubsystem = (Uptake) m_subsystemMap.get("Uptake");
      VisualizationUtils.updateMotorColor(
          m_uptake, uptakeSubsystem.getPercentOutput(), m_colorUptake);
    }
    if (m_subsystemMap.containsKey("Flywheel")) {
      var flywheelSubsystem = (Flywheel) m_subsystemMap.get("Flywheel");
      m_flywheel.update(RPM.of(flywheelSubsystem.getMotorSpeedRPM()));
    }
    if (m_subsystemMap.containsKey("Hood")) {
      var hoodSubsystem = (Hood) m_subsystemMap.get("Hood");
      m_shooterHood.update(Degrees.of(hoodSubsystem.getHoodAngleDegrees()).unaryMinus());
    }
    if (m_subsystemMap.containsKey("Climber")) {
      var climberSubsystem = (Climber) m_subsystemMap.get("Climber");
      m_climber.update(climberSubsystem.getHeight(), climberSubsystem.getVelocity());
    }
  }
}
