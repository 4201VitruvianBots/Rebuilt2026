package frc.team4201.lib.simulation.visualization;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Class to represent a top-down view of a swerve module using {@link Mechanism2d} */
public class SwerveModule2d implements AutoCloseable {
  private final MechanismLigament2d m_moduleLigament;
  private final Color8Bit m_ligamentColor = new Color8Bit(Color.kGray);
  private final String m_name;
  private final double m_moduleMaxSpeedMps;

  /**
   * Create a new {@link SwerveModule2d} instance
   *
   * @param name The name of the object (Must be unique across all {@link Mechanism2d} objects)
   * @param moduleMaxSpeedMps The max speed of the module
   */
  public SwerveModule2d(String name, double moduleMaxSpeedMps) {
    m_name = name;
    m_moduleMaxSpeedMps = moduleMaxSpeedMps;

    // Create a "line" to represent the swerve module.
    // We will use this to show the current direction/speed of the swerve module.
    m_moduleLigament = new MechanismLigament2d(m_name + "_direction", 0, 0);
  }

  /**
   * Update the {@link SwerveModule2d}'s visualization based on the current {@link
   * SwerveModuleState}
   *
   * @param state The {@link SwerveModuleState} of the swerve module
   */
  public void update(SwerveModuleState state) {
    m_moduleLigament.setLength(state.speedMetersPerSecond / (2 * m_moduleMaxSpeedMps) + 0.25);
    m_moduleLigament.setAngle(state.angle.getDegrees());

    // Update the ligament color based on the module's current speed for easier visualization
    VisualizationUtils.updateMotorColor(
        m_moduleLigament, state.speedMetersPerSecond, m_ligamentColor);

    m_moduleLigament.setColor(m_ligamentColor);
  }

  @Override
  public void close() throws Exception {
    m_moduleLigament.close();
  }
}
