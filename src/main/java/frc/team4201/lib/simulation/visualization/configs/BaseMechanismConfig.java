package frc.team4201.lib.simulation.visualization.configs;

import edu.wpi.first.wpilibj.util.Color8Bit;

/** Base configuration template for all mechanisms. */
public class BaseMechanismConfig {
  /** Mechanism name */
  public String m_name;

  /** Mechanism color */
  public Color8Bit m_color;

  /**
   * Create a new {@link BaseMechanismConfig}
   *
   * @param name The name of the mechanism
   */
  public BaseMechanismConfig(String name) {
    this(name, new Color8Bit());
  }

  /**
   * Create a new {@link BaseMechanismConfig}
   *
   * @param name The name of the mechanism
   * @param color The {@link Color8Bit} of the mechanism. Used for visualization.
   */
  public BaseMechanismConfig(String name, Color8Bit color) {
    m_name = name;
    m_color = color;
  }

  @Override
  public BaseMechanismConfig clone() {
    return new BaseMechanismConfig(m_name, m_color);
  }
}
