package frc.team4201.lib.simulation.visualization.configs;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.team4201.lib.simulation.visualization.Arm2d;

/** Configuration used for an {@link Arm2d} */
public class Arm2dConfig extends BaseMechanismConfig {
  /** Initial length of the arm */
  public Distance m_initialLength;

  /** Initial angle of the arm */
  public Angle m_initialAngle;

  /** Angle offset of the arm to its parent {@link MechanismLigament2d} */
  public Angle m_angleOffset = Degrees.of(0);

  /** Line width (thickness) of the arm. This is in pixels per inch of the Mechanism2d */
  public double m_lineWidth = 3;

  /** Max length of the arm */
  public Distance m_maxLength;

  /**
   * Initialize a config for the {@link Arm2d}.
   *
   * @param name Name of the mechanism
   */
  public Arm2dConfig(String name) {
    this(name, new Color8Bit(255, 255, 255), Degrees.of(0), Inches.of(12));
    this.m_lineWidth = 5;
  }

  /**
   * Initialize a config for the {@link Arm2d}.
   *
   * @param name Name of the mechanism
   * @param color {@link Color8Bit} to use for the mechanism
   * @param initialAngle Initial angle the Arm2d is at (0 degrees is parallel to ground)
   * @param initialLength Initial length of the Arm2d
   */
  public Arm2dConfig(String name, Color8Bit color, Angle initialAngle, Distance initialLength) {
    super(name, color);

    m_initialAngle = initialAngle;
    m_initialLength = initialLength;
    m_maxLength = m_initialLength;
  }

  /**
   * Set the angle offset of the {@link Arm2d} to its parent mechanism
   *
   * @param angleOffset The {@link Angle} offset of the Arm2d from its parent
   * @return this
   */
  public Arm2dConfig withAngleOffset(Angle angleOffset) {
    this.m_angleOffset = angleOffset;
    return this;
  }

  /**
   * Set the line width of the {@link Arm2d}.
   *
   * @param lineWidth The line width of the Arm. This is in pixels per inch of the {@link
   *     Mechanism2d}.
   * @return this
   */
  public Arm2dConfig withLineWidth(double lineWidth) {
    this.m_lineWidth = lineWidth;
    return this;
  }

  /**
   * Set the max length of the {@link Arm2d}.
   *
   * @param maxLength The max length of the Arm. Defaults to the Arm's initial length
   * @return this
   */
  public Arm2dConfig withMaxLength(Distance maxLength) {
    this.m_maxLength = maxLength;
    return this;
  }

  /**
   * Copy the {@link Arm2dConfig} object.
   *
   * @return {@link Arm2dConfig}
   */
  public Arm2dConfig clone() {
    return new Arm2dConfig(m_name, m_color, m_initialAngle, m_initialLength)
        .withLineWidth(m_lineWidth);
  }
}
