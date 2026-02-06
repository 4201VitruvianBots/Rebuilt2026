package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FLYWHEEL.Shot;
import frc.robot.Constants.SWERVE;
import frc.robot.constants.FIELD;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Vision;
import com.ctre.phoenix6.mechanisms.swerve.*;

public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap =
      new InterpolatingTreeMap<>(
          (startValue, endValue, q) ->
              InverseInterpolator.forDouble()
                  .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
          (startValue, endValue, t) ->
              new Shot(
                  Interpolator.forDouble()
                      .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                  Interpolator.forDouble()
                      .interpolate(startValue.hoodPosition, endValue.hoodPosition, t),
                  Interpolator.forDouble().interpolate(startValue.timeOfFlight, endValue.timeOfFlight, t)));

  static {
    distanceToShotMap.put(
        Meters.of(1.8086638318064376), new Shot(2175, 0.40, 10.0)); // Hood position is a placeholder
    distanceToShotMap.put(Meters.of(3.42), new Shot(2200, 0.19, 10.0));
    distanceToShotMap.put(Meters.of(6.00), new Shot(2300, 0.15, 10.0));
  }

  private final Flywheel m_flywheel;
  private final Vision m_vision;
  private final CommandSwerveDrivetrain m_swerveDrivetrain;
  private final DoubleSupplier m_throttleInput;
  private final DoubleSupplier m_turnInput;

  private Double kTeleP_Theta = 7.4;
  private Double kTeleD_Theta = 0.3;
  public static final double kTeleI_Theta = 0.0;

  private PIDController m_PidController =
    new PIDController(kTeleP_Theta, kTeleI_Theta, kTeleD_Theta);

  // private final ShooterHood m_shooterHood;

  Translation2d m_goal = new Translation2d();

  public Shoot(Flywheel flywheel, Vision vision, CommandSwerveDrivetrain swerveDrive, DoubleSupplier throttleInput, DoubleSupplier turnInput) {
    m_flywheel = flywheel;
    m_vision = vision;
    m_swerveDrivetrain = swerveDrive;
    m_throttleInput = throttleInput;
    m_turnInput = turnInput;
    // m_shooterHood = shooterHood;

    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Controls.isBlueAlliance()) {
      m_goal = FIELD.blueHub;
    } else {
      m_goal = FIELD.redHub;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shot shot = distanceToShotMap.get(m_vision.getDistancetoHub());
    Translation2d currentPose = m_swerveDrivetrain.getState().Pose.getTranslation();
    double robotToTargetDistance = m_goal.getDistance(currentPose);

    double PositionY = m_swerveDrivetrain.getState().Pose.getY();
    double PositionX = m_swerveDrivetrain.getState().Pose.getX();

    var chassisSpeeds = m_swerveDrivetrain.getState().Speeds;
    double VelocityY = chassisSpeeds.vyMetersPerSecond;
    double VelocityX = chassisSpeeds.vxMetersPerSecond;

    double AccelerationX = m_swerveDrivetrain.getPigeon2().getAccelerationX().getValueAsDouble();
    double AccelerationY = m_swerveDrivetrain.getPigeon2().getAccelerationY().getValueAsDouble();

    // Account for imparted velocity by robot to offset
    Pose2d lookaheadPose = m_swerveDrivetrain.getState().Pose;
    double timeOfFlight;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = shot.timeOfFlight;
      double offsetX = VelocityX * timeOfFlight;
      double offsetY = VelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              currentPose.plus(new Translation2d(offsetX, offsetY)),
              m_swerveDrivetrain.getState().Pose.getRotation());
      robotToTargetDistance = m_goal.getDistance(lookaheadPose.getTranslation());
    }

    double VelocityShoot = robotToTargetDistance / shot.timeOfFlight; // Previously 11.1 m/s

    double virtualGoalX = m_goal.getX() - VelocityShoot * (VelocityX + AccelerationX);
    double virtualGoalY = m_goal.getY() - VelocityShoot * (VelocityY + AccelerationY);

    SmartDashboard.putNumber("Goal X", virtualGoalX);
    SmartDashboard.putNumber("Goal Y", virtualGoalY);

    Translation2d movingGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

    Translation2d toMovingGoal = movingGoalLocation.minus(currentPose);

    double newDist = toMovingGoal.getDistance(new Translation2d());

    double getOffsetAngleDeg =
        Math.asin((VelocityY * PositionX + VelocityX * PositionY) / (newDist * robotToTargetDistance));

    var targetDelta = (m_swerveDrivetrain.getState().Pose.getTranslation().minus(m_goal).getAngle());

    // all of the logic for angle is above this Comment

    m_flywheel.setRPMOutputFOC(shot.shooterRPM);
    // m_shooterHood.setPosition(shot.hoodPosition);

    m_swerveDrivetrain.setChassisSpeedControl(
        new ChassisSpeeds(
            (m_throttleInput.getAsDouble()) * SWERVE.kMaxSpeedMetersPerSecond,
            (m_turnInput.getAsDouble()) * SWERVE.kMaxSpeedMetersPerSecond,
            m_PidController.calculate(
                    m_swerveDrivetrain.getState().Pose.getRotation().getRadians(),
                    targetDelta.getRadians() + getOffsetAngleDeg)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.setTorqueCurrentOutputFOC(Volts.of(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
