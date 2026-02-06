package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.constants.FIELD;
import frc.robot.constants.VISION;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class AutoAlignDrive extends Command {
  private final CommandSwerveDrivetrain m_swerveDrivetrain;
  private final Vision m_vision;
  Translation2d m_goal = new Translation2d();

  private Double kTeleP_Theta = 7.4;
  private Double kTeleD_Theta = 0.3;
  public static final double kTeleI_Theta = 0.0;

  private PIDController m_PidController =
      new PIDController(kTeleP_Theta, kTeleI_Theta, kTeleD_Theta);

  private final DoubleSupplier m_throttleInput;
  private final DoubleSupplier m_turnInput;

  /** Creates a new AutoAlign. */
  public AutoAlignDrive(
      CommandSwerveDrivetrain commandSwerveDrivetrain,
      Vision vision,
      DoubleSupplier throttleInput,
      DoubleSupplier turnInput) {
    m_swerveDrivetrain = commandSwerveDrivetrain;
    m_throttleInput = throttleInput;
    m_turnInput = turnInput;
    m_vision = vision;
    m_PidController.setTolerance(Units.degreesToRadians(2));
    m_PidController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kTeleP_Theta = m_vision.m_kPAutoAlignSubscriber.getAsDouble();
    kTeleD_Theta = m_vision.m_kDAutoAlignSubscriber.getAsDouble();
    m_PidController.reset();
    Rectangle2d m_goalZone;

    // If we're outside of our own zone, then we align to pass.
    if (m_vision.isInOpposingAllianceZone() || m_vision.isInNeutralZone()) {
      m_goalZone =
          Controls.isBlueAlliance()
              ? (m_vision.isInLeftHalf() ? FIELD.blueZoneLeft : FIELD.blueZoneRight)
              : (m_vision.isInLeftHalf() ? FIELD.redZoneLeft : FIELD.redZoneRight);
      m_goal = m_goalZone.getCenter().getTranslation();
      // If we're in our own zone, then we align to the hub
    } else {
      m_goal = Controls.isBlueAlliance() ? FIELD.blueAutoHub : FIELD.redAutoHub;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var setPoint = m_goal.minus(m_swerveDrivetrain.getState().Pose.getTranslation());
    var turnRate =
        m_PidController.calculate(
            m_swerveDrivetrain.getState().Pose.getRotation().getRadians(),
            setPoint.getAngle().getRadians());
    m_swerveDrivetrain.setChassisSpeedControl(
        new ChassisSpeeds(
            m_throttleInput.getAsDouble() * SWERVE.kMaxSpeedMetersPerSecond,
            m_turnInput.getAsDouble() * SWERVE.kMaxSpeedMetersPerSecond,
            turnRate
                - m_swerveDrivetrain.getState().Speeds.vyMetersPerSecond
                    * VISION.kVelocityCompensationConstant));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
