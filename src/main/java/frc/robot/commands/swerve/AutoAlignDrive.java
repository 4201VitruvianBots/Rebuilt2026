package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FIELD;
import frc.robot.constants.SWERVE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class AutoAlignDrive extends Command {
  private final CommandSwerveDrivetrain m_swerveDrivetrain;
  private final Vision m_vision;
  private Translation2d m_goal = Translation2d.kZero;

  private Double kTeleP_Theta = 7.4;
  private Double kTeleD_Theta = 0.3;
  public static final double kTeleI_Theta = 0.0;

  private final PIDController m_PidController =
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

    // If we're outside our own zone, then we align to pass.
    if (m_vision.isInOpposingAllianceSector() || m_vision.isInNeutralSector()) {
      // TODO: Add pass position as goal
      m_goal = Translation2d.kZero;
      // If we're in our own zone, then we align to the hub
    } else {
      m_goal = FIELD.HUB.GOAL.getTargetPosition().toTranslation2d();
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
            turnRate));
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
