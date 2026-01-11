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

  /** Change everything later when we know the size. */
  final MechanismRoot2d m_chassisRoot =
      m_robot.getRoot("chassisRoot", Inches.of(16).magnitude(), Inches.of(3.75).magnitude());

  private final MechanismRoot2d m_superStructureRoot =
      m_robot.getRoot("SuperStructureRoot", Inches.of(23.25).magnitude(), Inches.of(3).magnitude());


  /** Use a line (MechanismLigament2d) to represent the robot chassis */
  final MechanismLigament2d m_robotChassis2d =
      new MechanismLigament2d(
          "chassis2d",
          SWERVE.kWheelBase.in(Inches),
          0,
          Inches.of(2).magnitude() * pixelsPerInch,
          new Color8Bit(0, 127, 0));
          
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

  }
}
