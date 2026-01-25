package frc.team4201.lib.simulation.visualization.configs;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.team4201.lib.simulation.visualization.Elevator2d;
import frc.team4201.lib.simulation.visualization.Flywheel2d;

/** Configuration used for an {@link Elevator2d} */
public class Flywheel2dConfig extends BaseMechanismConfig {
  /** Initial radius of the flywheel. Default is 2 inches if not set */
  public Distance m_initialRadius;

  /** Max radius of the flywheel. Defaults to the value of m_initialRadius if not set */
  public Distance m_maxRadius;

  /** Line width (thickness) of the flywheel. This is in pixels of the Mechanism2d */
  public double m_lineWidth = 3;

  /**
   * Initialize a config for the {@link Flywheel2d}.
   *
   * @param name Name of the mechanism
   */
  public Flywheel2dConfig(String name) {
    this(name, new Color8Bit(255, 255, 255));
  }

  /**
   * Initialize a config for the {@link Flywheel2d}.
   *
   * @param name Name of the mechanism
   * @param color {@link Color8Bit} to use for the mechanism
   */
  public Flywheel2dConfig(String name, Color8Bit color) {
    this(name, color, Inches.of(2));
  }

  /**
   * Initialize a config for the {@link Flywheel2d}.
   *
   * @param name Name of the mechanism
   * @param color {@link Color8Bit} to use for the mechanism
   * @param initialRadius Initial length of the Flywheel2d
   */
  public Flywheel2dConfig(String name, Color8Bit color, Distance initialRadius) {
    super(name, color);

    m_initialRadius = initialRadius;
    m_maxRadius = m_initialRadius;
  }

  /**
   * Set the max radius of the {@link Flywheel2d}.
   *
   * @param maxRadius The max radius of the Flywheel. Defaults to the Flywheel's initial radius
   * @return this
   */
  public Flywheel2dConfig withMaxRadius(Distance maxRadius) {
    this.m_maxRadius = maxRadius;
    return this;
  }

  /**
   * Copy the {@link Flywheel2dConfig} object.
   *
   * @return {@link Flywheel2dConfig}
   */
  public Flywheel2dConfig clone() {
    return new Flywheel2dConfig(m_name, m_color, m_initialRadius);
  }
}
