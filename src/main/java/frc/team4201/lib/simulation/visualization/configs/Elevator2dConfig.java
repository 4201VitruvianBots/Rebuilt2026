package frc.team4201.lib.simulation.visualization.configs;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.team4201.lib.simulation.visualization.Elevator2d;
import frc.team4201.lib.simulation.visualization.VisualizationUtils.ELEVATOR_TYPE;

/** Configuration used for an {@link Elevator2d} */
public class Elevator2dConfig extends BaseMechanismConfig {
  /** Number of stages the elevator uses. Used for visualization */
  public int m_numberOfStages;

  /** Initial length of the elevator */
  public Distance m_initialLength;

  /** Max lengths of each stage (For CONTINUOUS elevators) */
  public Distance[] m_stageMaxLengths;

  /** Angle offset of the elevator to its parent {@link MechanismLigament2d} */
  public Angle m_angleOffset;

  /** Offset of the elevator's superStructure */
  public Distance m_superStructureOffset;

  /** {@link Elevator2d} type (for visualization). Default is CASCADE */
  public ELEVATOR_TYPE m_type = ELEVATOR_TYPE.CASCADE;

  /** {@link Color8Bit} to distinguish elevator stages */
  public Color8Bit[] m_stageColors;

  /** Line width (thickness) of the elevator. This is in pixels of the Mechanism2d */
  public double m_lineWidth = 3;

  /**
   * Initialize a config for the {@link Elevator2d}.
   *
   * @param name Name of the mechanism
   */
  public Elevator2dConfig(String name) {
    this(name, new Color8Bit(0, 128, 0), Inches.of(12), Degrees.of(90));
  }

  /**
   * Initialize a config for the {@link Elevator2d}.
   *
   * @param name Name of the mechanism
   * @param color {@link Color8Bit} to use for the mechanism
   * @param initialLength Initial length of the Elevator2d
   */
  public Elevator2dConfig(String name, Color8Bit color, Distance initialLength) {
    this(name, color, initialLength, Degrees.of(0));
  }

  /**
   * Initialize a config for the {@link Elevator2d}.
   *
   * @param name Name of the mechanism
   * @param color {@link Color8Bit} to use for the mechanism
   * @param initialLength Initial length of the Elevator2d
   * @param initialAngleOffset Initial angle offset.
   */
  public Elevator2dConfig(
      String name, Color8Bit color, Distance initialLength, Angle initialAngleOffset) {
    this(name, color, initialLength, initialAngleOffset, initialLength);
  }

  /**
   * Initialize a config for the {@link Elevator2d}.
   *
   * @param name Name of the mechanism
   * @param color {@link Color8Bit} to use for the mechanism
   * @param initialLength Initial length of the Elevator2d
   * @param initialAngleOffset Initial angle offset.
   * @param maxStageLengths Maximum extension of each stage. Defaults to the initialLength
   */
  public Elevator2dConfig(
      String name,
      Color8Bit color,
      Distance initialLength,
      Angle initialAngleOffset,
      Distance... maxStageLengths) {
    super(name, color);

    m_initialLength = initialLength;
    m_angleOffset = initialAngleOffset;
    m_stageMaxLengths = maxStageLengths;
    m_numberOfStages = maxStageLengths.length;
    m_stageColors = new Color8Bit[m_numberOfStages];
    for (int i = 0; i < m_numberOfStages; i++) {
      m_stageColors[i] = m_color;
    }
  }

  /**
   * Set the angle offset of the {@link Elevator2d} to its parent mechanism
   *
   * @param angleOffset The {@link Angle} offset of the Elevator from its parent
   * @return this
   */
  public Elevator2dConfig withAngleOffset(Angle angleOffset) {
    this.m_angleOffset = angleOffset;
    return this;
  }

  /**
   * Set the super structure offset of the {@link Elevator2d} to its parent mechanism
   *
   * @param lengthOffset The {@link Distance} of the super structure when the elevator's height
   *     reads 0.
   * @return this
   */
  public Elevator2dConfig withSuperStructureOffset(Distance lengthOffset) {
    this.m_superStructureOffset = lengthOffset;
    return this;
  }

  /**
   * Set the {@link Elevator2d} type (For visualization)
   *
   * @param type The {@link ELEVATOR_TYPE}
   * @return this
   */
  public Elevator2dConfig withElevatorType(ELEVATOR_TYPE type) {
    this.m_type = type;
    return this;
  }

  /**
   * Set the line width of the {@link Elevator2d}.
   *
   * @param lineWidth The line width of the elevator. This is in pixels per inch of the {@link
   *     Mechanism2d}.
   * @return this
   */
  public Elevator2dConfig withLineWidth(double lineWidth) {
    this.m_lineWidth = lineWidth;
    return this;
  }

  /**
   * Set the max length for each stage for the {@link Elevator2d} (For visualization). Must be set
   * for CONTINUOUS elevators.
   *
   * @param stageMaxLengths The max {@link Distance} each stage can can reach.
   * @return this
   */
  public Elevator2dConfig withStageMaxLengths(Distance... stageMaxLengths) {
    this.m_stageMaxLengths = stageMaxLengths;
    this.m_numberOfStages = m_stageMaxLengths.length;
    return this;
  }

  /**
   * Set the colors for each stage for the {@link Elevator2d} (For visualization). Must be set for
   * CONTINUOUS elevators.
   *
   * @param colors The {@link Color8Bit} for each stage.
   * @return this
   */
  public Elevator2dConfig withStageColors(Color8Bit... colors) {
    this.m_stageColors = colors;
    return this;
  }

  /**
   * Copy the {@link Elevator2dConfig} object.
   *
   * @return {@link Elevator2dConfig}
   */
  public Elevator2dConfig clone() {
    return new Elevator2dConfig(m_name, m_color, m_initialLength, m_angleOffset, m_stageMaxLengths)
        .withLineWidth(m_lineWidth)
        .withSuperStructureOffset(m_superStructureOffset)
        .withStageColors(m_stageColors);
  }
}
