package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.ELEVATOR;
import frc.robot.constants.ENDEFFECTOR.PIVOT;
import frc.robot.constants.GROUND;
import frc.robot.constants.SWERVE;
import frc.robot.subsystems.*;
import java.util.HashMap;
import java.util.Map;
import org.team4201.codex.simulation.visualization.Arm2d;
import org.team4201.codex.simulation.visualization.Elevator2d;
import org.team4201.codex.simulation.visualization.Flywheel2d;
import org.team4201.codex.simulation.visualization.configs.Arm2dConfig;
import org.team4201.codex.simulation.visualization.configs.Elevator2dConfig;
import org.team4201.codex.simulation.visualization.configs.Flywheel2dConfig;

/**
 * Class to handle all Mechanism2d updates. The width/height of the Mechanism2d is scaled based on
 * the window in Glass/SmartDashboard. For consistency, we should just use Inches.
 */
public class Robot2d {

  /**
   * Image dimensions 657/830, which ends up being 29 pixels/2 inches. Use this to scale the
   * lineWidth of MechanismLigament2d appropriately
   */
  // TODO: No longer accurate? Review/fix
  private final double pixelsPerInch = 29.0 / 2.0;

  private final Distance robotCanvasX = Inches.of(45.31034);
  private final Distance robotCanvasY = Inches.of(57.24138 + 20.0);

  private final Mechanism2d m_robot =
      new Mechanism2d(robotCanvasX.magnitude(), robotCanvasY.magnitude());

  /** Declare a single point from the main Mechanism2d to attach the chassis */
  final MechanismRoot2d m_chassisRoot =
      m_robot.getRoot("chassisRoot", Inches.of(16).magnitude(), Inches.of(3.75).magnitude());

  private final MechanismRoot2d m_superStructureRoot =
      m_robot.getRoot("SuperStructureRoot", Inches.of(23.25).magnitude(), Inches.of(3).magnitude());
  final MechanismLigament2d m_superStructure2d =
      new MechanismLigament2d(
          "superStructure2d",
          ELEVATOR.superStructureHeight.in(Inches),
          90,
          Inches.of(2).magnitude() * pixelsPerInch,
          new Color8Bit(0, 127, 0));

  /** Use a line (MechanismLigament2d) to represent the robot chassis */
  final MechanismLigament2d m_robotChassis2d =
      new MechanismLigament2d(
          "chassis2d",
          SWERVE.kWheelBase.in(Inches),
          0,
          Inches.of(2).magnitude() * pixelsPerInch,
          new Color8Bit(0, 127, 0));

  /** Declare a single point from the main Mechanism2d to attach the Elevator2d */
  private final MechanismRoot2d m_elevatorRoot =
      m_robot.getRoot("ElevatorRoot", Inches.of(23.25).magnitude(), Inches.of(3).magnitude());

  /** Create an Elevator2d to represent an elevator, and then attach it to the m_elevatorRoot */
  private final Elevator2d m_elevator2d =
      new Elevator2d(
          new Elevator2dConfig("Elevator2d", new Color8Bit(0, 128, 0), Inches.of(0), Degrees.of(90))
              .withLineWidth(Inches.of(2).magnitude() * pixelsPerInch)
              .withSuperStructureOffset(Inches.of(3))
              .withStageMaxLengths(ELEVATOR.upperLimit)
              .withStageColors(new Color8Bit(0, 0, 255)),
          m_elevatorRoot);

  // TODO: Clean this up?
  private final MechanismLigament2d m_endEffectorAttachment =
      m_elevator2d
          .getLastStageLigament()
          .append(
              new MechanismLigament2d(
                  "endEffectorAttachment2d",
                  Inches.of(11.75).magnitude(),
                  36.11,
                  Inches.of(2).magnitude() * pixelsPerInch,
                  new Color8Bit(0, 127, 0)));

  /** Create an Arm2d to represent the endEffector */
  private final Arm2d m_endEffector2d =
      new Arm2d(
          new Arm2dConfig(
                  "EndEffector2d",
                  new Color8Bit(0, 255, 255),
                  PIVOT.startingAngle.minus(Degrees.of(25.676)),
                  PIVOT.baseLength)
              .withLineWidth(Inches.of(2).magnitude() * pixelsPerInch),
          m_endEffectorAttachment);

  // TODO: Just turn this into a ligament2d to avoid complexity
  /** Another Arm2d to represent the other half of the endEffector */
  private final Arm2d m_endEffectorBack =
      new Arm2d(
          new Arm2dConfig(
                  "EndEffectorBack2d",
                  new Color8Bit(0, 255, 255),
                  PIVOT.startingAngle.minus(Degrees.of(25.676)).plus(Degrees.of(90)),
                  PIVOT.baseLength)
              .withLineWidth(Inches.of(2).magnitude() * pixelsPerInch),
          m_endEffectorAttachment);

  final MechanismLigament2d m_endEffectorIntake =
      m_endEffector2d
          .getLigament()
          .append(
              new MechanismLigament2d(
                  "endEffectorIntake2d",
                  Inches.of(9).magnitude(),
                  -47.5,
                  Inches.of(1.9675).magnitude() * pixelsPerInch,
                  new Color8Bit(0, 0, 255)));

  final Flywheel2d m_endEffectorRoller =
      new Flywheel2d(new Flywheel2dConfig("endEffectorRoller2d"), m_endEffectorIntake);

  /** Declare a single point from the main Mechanism2d to attach the Elevator2d */
  private final MechanismRoot2d m_groundIntakePivotRoot =
      m_robot.getRoot("GroundIntakePivotRoot", Inches.of(30).magnitude(), Inches.of(6).magnitude());

  /** Create an Arm2d to represent the GroundIntakePivot */
  private final Arm2d m_groundIntakePivot2d =
      new Arm2d(
          new Arm2dConfig(
                  "GroundIntakePivot2d",
                  new Color8Bit(255, 50, 50),
                  GROUND.PIVOT.startingAngle,
                  GROUND.PIVOT.pivotLength)
              .withLineWidth(Inches.of(1).magnitude() * pixelsPerInch),
          m_groundIntakePivotRoot);

  /** Map of subsystems for Robot2d to update */
  private final Map<String, Subsystem> m_subsystemMap = new HashMap<>();

  public Robot2d() {
    // Attach the robotChassis to the chassisRoot
    m_chassisRoot.append(m_robotChassis2d);
    m_superStructureRoot.append(m_superStructure2d);

    // Publish Robot2d to SmartDashboard
    SmartDashboard.putData("Robot2d", m_robot);

    // For simulation, create a sub-mechanism display for each mechanism.
    // Avoid doing this with the real robot to reduce bandwidth usage.
    if (RobotBase.isSimulation()) {
      m_elevator2d.generateSubDisplay();
    }
  }

  public void registerSubsystem(Subsystem subsystem) {
    if (subsystem != null) {
      m_subsystemMap.put(subsystem.getName(), subsystem);
    } else {
      DriverStation.reportWarning("[Robot2d] Attempting to register null subsystem!", true);
    }
  }

  /** Function to update all mechanisms on Robot2d. This should be called periodically. */
  public void updateRobot2d() {
    if (m_subsystemMap.containsKey("Elevator")) {
      var elevatorSubsystem = (Elevator) m_subsystemMap.get("Elevator");
      m_elevator2d.update(elevatorSubsystem.getHeight());
      m_elevator2d.update(elevatorSubsystem.getHeight(), elevatorSubsystem.getVelocity());
    }

    if (m_subsystemMap.containsKey("EndEffectorPivot")) {
      var endEffectorPivotSubsystem = (EndEffectorPivot) m_subsystemMap.get("EndEffectorPivot");
      // Visually, this will go opposite of the actual angle, so we just negate it here so it looks
      // correct
      m_endEffector2d.update(
          endEffectorPivotSubsystem.getCANcoderAngle().minus(Degrees.of(25.676)).unaryMinus());
      m_endEffectorBack.update(
          endEffectorPivotSubsystem
              .getCANcoderAngle()
              .minus(Degrees.of(25.676))
              .unaryMinus()
              .plus(Degrees.of(90)));
    }

    if (m_subsystemMap.containsKey("EndEffector")) {
      var endEffectorSubsystem = (EndEffector) m_subsystemMap.get("EndEffector");

      m_endEffectorRoller.update(endEffectorSubsystem.getVelocity());
    }

    if (m_subsystemMap.containsKey("GroundPivot")) {
      var groundPivotSubsystem = (GroundPivot) m_subsystemMap.get("GroundPivot");
      m_groundIntakePivot2d.update(Degrees.of(0).minus(groundPivotSubsystem.getAngle()));
    }

    if (m_subsystemMap.containsKey("GroundIntake")) {
      var groundIntakeSubsystem = (GroundIntake) m_subsystemMap.get("GroundIntake");
    }

    if (m_subsystemMap.containsKey("Climber")) {
      var climberSubsystem = (Climber) m_subsystemMap.get("Climber");
    }
  }
}
