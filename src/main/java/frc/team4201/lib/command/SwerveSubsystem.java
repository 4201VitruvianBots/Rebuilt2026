package frc.team4201.lib.command;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.SWERVE;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredDevice;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredSubsystem;
import frc.team4201.lib.utils.TrajectoryUtils;
import frc.team4201.lib.vision.LimelightHelpers;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class SwerveSubsystem extends CommandSwerveDrivetrain
    implements Subsystem, MonitoredSubsystem {
  public enum MOTOR_TYPE {
    ALL,
    DRIVE,
    STEER
  }

  @MonitoredDevice(
      name = "Front Left Drive",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore",
      skipDefaultInit = true)
  @Logged(name = "Front Left Drive", importance = Logged.Importance.INFO)
  private final TalonFX frontLeftDrive = getModule(0).getDriveMotor();

  @MonitoredDevice(
      name = "Front Left Steer",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore",
      skipDefaultInit = true)
  @Logged(name = "Front Left Steer", importance = Logged.Importance.INFO)
  private final TalonFX frontLeftSteer = getModule(0).getSteerMotor();

  @MonitoredDevice(
      name = "Front Left Encoder",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore",
      skipDefaultInit = true)
  @Logged(name = "Front Left Encoder", importance = Logged.Importance.INFO)
  private final CANcoder frontLeftEncoder = getModule(1).getEncoder();

  @MonitoredDevice(
      name = "Front Right Drive",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore",
      skipDefaultInit = true)
  @Logged(name = "Front Right Drive", importance = Logged.Importance.INFO)
  private final TalonFX frontRightDrive = getModule(1).getDriveMotor();

  @MonitoredDevice(
      name = "Front Right Steer",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore",
      skipDefaultInit = true)
  @Logged(name = "Front Right Steer", importance = Logged.Importance.INFO)
  private final TalonFX frontRightSteer = getModule(1).getSteerMotor();

  @MonitoredDevice(
      name = "Front Right Encoder",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore",
      skipDefaultInit = true)
  @Logged(name = "Front Right Encoder", importance = Logged.Importance.INFO)
  private final CANcoder frontRightEncoder = getModule(1).getEncoder();

  @MonitoredDevice(
      name = "Back Right Drive",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore",
      skipDefaultInit = true)
  @Logged(name = "Back Right Drive", importance = Logged.Importance.INFO)
  private final TalonFX backRightDrive = getModule(2).getDriveMotor();

  @MonitoredDevice(
      name = "Back Right Steer",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore",
      skipDefaultInit = true)
  @Logged(name = "Back Right Steer", importance = Logged.Importance.INFO)
  private final TalonFX backRightSteer = getModule(2).getSteerMotor();

  @MonitoredDevice(
      name = "Back Right Encoder",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore",
      skipDefaultInit = true)
  @Logged(name = "Back Right Encoder", importance = Logged.Importance.INFO)
  private final CANcoder backRightEncoder = getModule(2).getEncoder();

  @MonitoredDevice(
      name = "Back Left Drive",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore",
      skipDefaultInit = true)
  @Logged(name = "Back Left Drive", importance = Logged.Importance.INFO)
  private final TalonFX backLeftDrive = getModule(3).getDriveMotor();

  @MonitoredDevice(
      name = "Back Left Steer",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore",
      skipDefaultInit = true)
  @Logged(name = "Back Left Steer", importance = Logged.Importance.INFO)
  private final TalonFX backLeftSteer = getModule(3).getSteerMotor();

  @MonitoredDevice(
      name = "Back Left Encoder",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore",
      skipDefaultInit = true)
  @Logged(name = "Back Left Encoder", importance = Logged.Importance.INFO)
  private final CANcoder backLeftEncoder = getModule(3).getEncoder();

  @MonitoredDevice(
      name = "Pigeon2",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "canivore")
  @Logged(name = "Pigeon2", importance = Logged.Importance.INFO)
  private final Pigeon2 pigeon2 = getPigeon2();

  private final TalonFX[] driveMotors = {
    frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive
  };

  private final TalonFX[] steerMotors = {
    frontLeftSteer, frontRightSteer, backRightSteer, backLeftSteer
  };

  private TrajectoryUtils m_trajectoryUtils;

  SwerveRequest.FieldCentricFacingAngle m_driveWithHeadingRequest =
      new SwerveRequest.FieldCentricFacingAngle();

  SwerveRequest.SwerveDriveBrake m_holdPositionRequest = new SwerveRequest.SwerveDriveBrake();

  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
  }

  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
  }

  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);

    try {
      m_trajectoryUtils =
          new TrajectoryUtils(
              this, new TrajectoryUtils.TrajectoryUtilsConfig().withResetPoseOnAuto(true));
    } catch (Exception ex) {
      DriverStation.reportError("Failed to configure TrajectoryUtils", ex.getStackTrace());
    }
  }

  public void setNeutralMode(MOTOR_TYPE type, NeutralModeValue neutralModeValue) {
    switch (type) {
      case ALL -> {
        for (int i = 0; i < driveMotors.length; i++) {
          driveMotors[i].setNeutralMode(neutralModeValue);
          steerMotors[i].setNeutralMode(neutralModeValue);
        }
      }
      case DRIVE -> {
        for (var motor : driveMotors) {
          motor.setNeutralMode(neutralModeValue);
        }
      }
      case STEER -> {
        for (var motor : steerMotors) {
          motor.setNeutralMode(neutralModeValue);
        }
      }
    }
  }

  public Angle getGyroYaw() {
    return getPigeon2().getYaw().getValue();
  }

  public AngularVelocity getGyroYawRate() {
    return getPigeon2().getAngularVelocityZWorld().getValue().unaryMinus();
  }

  public void resetGyro(Angle angle) {
    getPigeon2().setYaw(angle);
  }

  public TrajectoryUtils getTrajectoryUtils() {
    return m_trajectoryUtils;
  }

  /**
   * Function to get the module locations as a {@link Translation2d} array
   *
   * @return Translation2d[]
   */
  //  Translation2d[] getModuleLocations();

  /**
   * Function to get the PathPlanner {@link RobotConfig}
   *
   * @return RobotConfig
   */
  public RobotConfig getAutoRobotConfig() {
    try {
      return RobotConfig.fromGUISettings();
    } catch (IOException e) {
      DriverStation.reportWarning(
          "[SwerveDrive] Could not load RobotConfig for autos!", e.getStackTrace());
      throw new RuntimeException(e);
    } catch (ParseException e) {
      DriverStation.reportWarning(
          "[SwerveDrive] Could not parse RobotConfig for autos!", e.getStackTrace());
      throw new RuntimeException(e);
    }
  }

  /**
   * Function to get the translation {@link PIDConstants} for PathPlanner
   *
   * @return PIDConstants
   */
  public PIDConstants getAutoTranslationPIDConstants() {
    return SWERVE.kTranslationPID;
  }

  /**
   * Function to get the rotation {@link PIDConstants} for PathPlanner
   *
   * @return PIDConstants
   */
  public PIDConstants getAutoRotationPIDConstants() {
    return SWERVE.kRotationPID;
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setControl(m_pathApplyRobotSpeeds.withSpeeds(chassisSpeeds));
  }

  public void setChassisSpeedsWithHeading(
      LinearVelocity velocityX, LinearVelocity velocityY, Rotation2d headingTarget) {
    setControl(
        m_driveWithHeadingRequest
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withTargetDirection(headingTarget));
  }

  /**
   * Function for PathPlanner to control the robot's motion in auto.
   *
   * @param chassisSpeeds WPILib's {@link ChassisSpeeds}
   * @param driveFeedforwards PathPlanner's {@link DriveFeedforwards}
   */
  public void setChassisSpeedsAuto(
      ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFeedforwards) {
    setControl(
        m_pathApplyRobotSpeeds
            .withSpeeds(chassisSpeeds)
            .withWheelForceFeedforwardsX(driveFeedforwards.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(driveFeedforwards.robotRelativeForcesYNewtons()));
  }

  public void addVisionMeasurement(LimelightHelpers.PoseEstimate poseEstimate) {
    addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
  }
}
