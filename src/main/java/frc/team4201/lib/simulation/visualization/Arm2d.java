package frc.team4201.lib.simulation.visualization;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.team4201.lib.simulation.visualization.configs.Arm2dConfig;

/** Class to represent an arm using {@link Mechanism2d} */
public class Arm2d implements AutoCloseable {
  private final Arm2dConfig m_config;
  private final MechanismLigament2d m_arm2d;
  private MechanismObject2d m_parentObject;
  private Arm2d m_subArm2d;

  /**
   * Create a new {@link Arm2d} instance
   *
   * @param config The {@link Arm2dConfig} that defines the Arm2d parameters
   */
  public Arm2d(Arm2dConfig config) {
    this(config, null);
  }

  /**
   * Create a new {@link Arm2d} instance
   *
   * @param config The {@link Arm2dConfig} that defines the Arm2d parameters
   * @param parentObject The {@link MechanismObject2d} (Either a {@link MechanismRoot2d} or {@link
   *     MechanismLigament2d})the Arm2d attaches to
   */
  public Arm2d(Arm2dConfig config, MechanismObject2d parentObject) {
    m_config = config;

    // Create a "line" to represent the arm.
    // We will use this to show its current position
    m_arm2d =
        new MechanismLigament2d(
            m_config.m_name,
            m_config.m_initialLength.in(Inches),
            m_config.m_initialAngle.in(Degrees),
            m_config.m_lineWidth,
            m_config.m_color);

    if (parentObject != null) {
      m_parentObject = parentObject;

      m_parentObject.append(m_arm2d);
    }
  }

  /**
   * Make a copy of the {@link Arm2d} on its own Mechanism2d display separate from the main robot
   */
  public void generateSubDisplay() {
    var defaultMechanism2dDimensions =
        new Translation2d(m_config.m_maxLength.in(Inches), m_config.m_maxLength.in(Inches))
            .times(1.2);
    generateSubDisplay(defaultMechanism2dDimensions);
  }

  /**
   * Make a copy of the {@link Arm2d} on its own Mechanism2d display separate from the main robot
   *
   * @param displayDimensions A {@link Translation2d} that contains the X/Y dimensions of the {@link
   *     Mechanism2d}
   */
  public void generateSubDisplay(Translation2d displayDimensions) {
    var defaultMechanism2dRootPosition = displayDimensions.times(0.5);
    generateSubDisplay(displayDimensions, defaultMechanism2dRootPosition);
  }

  /**
   * Make a copy of the {@link Arm2d} on its own Mechanism2d display separate from the main robot
   *
   * @param displayDimensions A {@link Translation2d} that contains the X/Y dimensions of the {@link
   *     Mechanism2d}
   * @param rootPosition A {@link Translation2d} that contains the X/Y position of the {@link
   *     Mechanism2d}
   */
  public void generateSubDisplay(Translation2d displayDimensions, Translation2d rootPosition) {
    Arm2dConfig armSubConfig = m_config.clone();
    armSubConfig.m_name = armSubConfig.m_name + "_sub";
    m_subArm2d = new Arm2d(armSubConfig);

    Mechanism2d subArmDisplay = new Mechanism2d(displayDimensions.getX(), displayDimensions.getY());
    subArmDisplay
        .getRoot(m_subArm2d + "Root", rootPosition.getX(), rootPosition.getY())
        .append(m_subArm2d.getLigament());
    ;

    SmartDashboard.putData(armSubConfig.m_name, subArmDisplay);
  }

  /**
   * Get the {@link Arm2d}'s {@link Arm2dConfig}
   *
   * @return {@link Arm2dConfig}
   */
  public Arm2dConfig getConfig() {
    return m_config;
  }

  /**
   * Get the {@link Arm2d}'s {@link MechanismLigament2d}
   *
   * @return {@link MechanismLigament2d}
   */
  public MechanismLigament2d getLigament() {
    return m_arm2d;
  }

  /**
   * Get the {@link Arm2d}'s parent {@link MechanismObject2d}. Returns null if it doesn't exist.
   *
   * @return {@link MechanismObject2d}
   */
  public MechanismObject2d getParentObject() {
    return m_parentObject;
  }

  /**
   * Get the {@link Arm2d}'s subMechanism. For displaying the mechanism by itself.
   *
   * @return {@link Arm2d}
   */
  public Arm2d getArmSubMechanism() {
    return m_subArm2d;
  }

  /**
   * Set the {@link Arm2d}'s {@link Angle}.
   *
   * @param angle Angle of the Arm2d
   */
  public void setAngle(Angle angle) {
    setAngle(angle, angle);
  }

  /**
   * Set the {@link Arm2d}'s {@link Angle}. This function is used if you want a separate orientation
   * for the standalone display of the Arm2d.
   *
   * @param angle Angle of the Arm2d
   * @param subAngle Angle of the Arm2d's subMechanism.
   */
  public void setAngle(Angle angle, Angle subAngle) {
    m_arm2d.setAngle(angle.in(Degrees));

    if (m_subArm2d != null) {
      m_subArm2d.getLigament().setAngle(subAngle.in(Degrees));
    }
  }

  /**
   * Update the {@link Arm2d}'s position relative to its attachment point
   *
   * @param angle The angle to set the Arm2d to.
   */
  public void update(Angle angle) {
    update(angle, RotationsPerSecond.of(0));
  }

  /**
   * Update the {@link Arm2d}'s position relative to its attachment point
   *
   * @param angle The angle to set the Arm2d to.
   * @param velocity The velocity of the Arm2d. Used to change the color of the Arm2d for
   *     visualization.
   */
  public void update(Angle angle, AngularVelocity velocity) {
    update(angle, RotationsPerSecond.of(0), m_config.m_initialLength);
  }

  /**
   * Update the {@link Arm2d}'s position relative to its attachment point
   *
   * @param angle The angle to set the Arm2d to.
   * @param velocity The velocity of the Arm2d. Used to change the color of the Arm2d for
   *     visualization.
   * @param length The length of the Arm2d. By default, it will use its initial length.
   */
  public void update(Angle angle, AngularVelocity velocity, Distance length) {
    m_arm2d.setLength(length.in(Inches));
    m_arm2d.setAngle(m_config.m_angleOffset.minus(angle).in(Degrees));

    // Update the ligament color based on the module's current speed for easier visualization
    VisualizationUtils.updateMotorColor(m_arm2d, velocity.in(RotationsPerSecond), m_config.m_color);

    if (m_subArm2d != null) {
      m_subArm2d.update(angle, velocity, length);
    }
  }

  @Override
  public void close() throws Exception {
    m_arm2d.close();

    if (m_subArm2d != null) {
      m_subArm2d.close();
    }
  }
}
