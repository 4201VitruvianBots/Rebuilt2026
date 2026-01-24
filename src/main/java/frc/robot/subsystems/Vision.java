package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VISION.CAMERA_SERVER;
// import frc.team4201.lib.simulation.LimelightSim;
import frc.team4201.lib.simulation.FieldSim;
import frc.team4201.lib.vision.LimelightHelpers;

public class Vision extends SubsystemBase {
  private CommandSwerveDrivetrain m_swerveDriveTrain;
  private FieldSim m_fieldSim;
  private Translation2d m_goal = new Translation2d();
  // TODO: Re-add this
  //   private LimelightSim visionSim;
  private Controls m_controls;

  private boolean m_localized;

  private boolean m_useLeftTarget;

  private Pose2d nearestObjectPose = Pose2d.kZero;
  private final Pose2d[] robotToTarget = {Pose2d.kZero, Pose2d.kZero};
  private boolean lockTarget = false;
  private boolean hasInitialPose = false;
  // NetworkTables publisher setup
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("LimelightPoseEstimate");

  private final DoublePublisher estTimeStamp = table.getDoubleTopic("estTimeStamp").publish();

  private final StructPublisher<Pose2d> estPoseLLR =
      table.getStructTopic("estPoseLLR", Pose2d.struct).publish();

  private final StructPublisher<Pose2d> estPoseLLL =
      table.getStructTopic("estPoseLLL", Pose2d.struct).publish();

  public Vision(Controls controls) {
    m_controls = controls;

    // Port Forwarding to access limelight web UI on USB Ethernet
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, CAMERA_SERVER.limelightR.toString(), port);
      PortForwarder.add(port + 10, CAMERA_SERVER.limelightL.toString(), port);
    }
  }

  public void registerSwerveDrive(CommandSwerveDrivetrain swerveDriveTrain) {
    m_swerveDriveTrain = swerveDriveTrain;
  }

  public void registerFieldSim(FieldSim fieldSim) {
    m_fieldSim = fieldSim;
  }

  public void setLeftTarget(boolean value) {
    m_useLeftTarget = value;
  }

  @Logged(name = "Left Target", importance = Logged.Importance.CRITICAL)
  public boolean isTargetingLeft() {
    return m_useLeftTarget;
  }

  @Logged(name = "Right Target", importance = Logged.Importance.CRITICAL)
  public boolean isTargetingRight() {
    return !m_useLeftTarget;
  }

  //   private void updateAngleToHub() {
  //   if (m_swerveDriveTrain != null) {
  //     if (DriverStation.isDisabled()) {
  //       if (DriverStation.isAutonomous()) {
  //         m_goal = Controls.isRedAlliance() ? FIELD.redAutoHub : FIELD.blueAutoHub;
  //       } else {
  //         m_goal = Controls.isRedAlliance() ? FIELD.redHub : FIELD.blueHub;
  //       }
  //     }
  //     if(DriverStation.isAutonomous()){
  //       m_swerveDriveTrain.setAngleToHub(
  //           m_swerveDriveTrain
  //               .getState()
  //               .Pose
  //              .getTranslation()
  //              .minus(m_goal)
  //              .getAngle());
  //     }
  //   }
  // }

  public boolean getInitialLocalization() {
    return m_localized;
  }

  public void resetInitialLocalization() {
    m_localized = false;

    // Set Swerve Pose to (0, 0) to reset it
    if (m_swerveDriveTrain != null) {
      m_swerveDriveTrain.resetPose(Pose2d.kZero);
    }
  }

  /**
   * Process measurements from a limelight. Return true if the given vision measurement is used,
   * otherwise return false to indicate that it was rejected.
   */
  public boolean processLimelight(String limelightName, StructPublisher<Pose2d> posePublisher) {
    if (DriverStation.isDisabled()) {
      // TODO: Determine if we change IMUMode to 0 when not disabled for MegaTag2
      LimelightHelpers.SetIMUMode(limelightName, 1);

      // Only use Reef AprilTags for localization
      // TODO: Update code values before using this
      //      LimelightHelpers.setCameraPose_RobotSpace(
      //          "limelight-f",
      //          VISION.limelightFPosition.getX(),
      //          VISION.limelightFPosition.getY(),
      //          VISION.limelightFPosition.getZ(),
      //          VISION.limelightFPosition.getRotation().getMeasureX().in(Degrees),
      //          VISION.limelightFPosition.getRotation().getMeasureY().in(Degrees),
      //          VISION.limelightFPosition.getRotation().getMeasureZ().in(Degrees));
      //      LimelightHelpers.setCameraPose_RobotSpace(
      //          "limelight-b",
      //          VISION.limelightBPosition.getX(),
      //          VISION.limelightBPosition.getY(),
      //          VISION.limelightBPosition.getZ(),
      //          VISION.limelightBPosition.getRotation().getMeasureX().in(Degrees),
      //          VISION.limelightBPosition.getRotation().getMeasureY().in(Degrees),
      //          VISION.limelightBPosition.getRotation().getMeasureZ().in(Degrees));
    }

    LimelightHelpers.SetRobotOrientation(
        limelightName,
        m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.PoseEstimate limelightMeasurement;
    if (DriverStation.isDisabled()) {
      // Use MegaTag1 when the robot is disabled to set the initial robot pose
      limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    } else {
      // Use MegaTag2 when the robot is running for more accurate pose updates
      limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    if (limelightMeasurement == null) {
      if (RobotBase.isReal())
        DriverStation.reportWarning(limelightName + " is not connected", true);
      return false;
    } else {
      // Filter out bad AprilTag vision estimates for both MegaTag1 and MegaTag2
      if (limelightMeasurement.timestampSeconds == 0) {
        return false;
      } else if (limelightMeasurement.pose.getTranslation().equals(Translation2d.kZero)) {
        return false;
      } else if (limelightMeasurement.tagCount == 0) {
        return false;
      }

      if (!limelightMeasurement.isMegaTag2) {
        // Filter out bad AprilTag vision estimates for MegaTag1
        // TODO: Check 1 tag from center?
        if (limelightMeasurement.tagCount < 1) {
          return false;
        }

        hasInitialPose = true;
        // Set Standard Deviations for MegaTag1
        m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
      } else {
        // Ignore MegaTag2 updates if the robot is spinning too fast
        if (m_swerveDriveTrain.getGyroYawRate().abs(DegreesPerSecond) > 720.0) {
          return false;
        }

        // Set Standard Deviations for MegaTag2
        m_swerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.4, .4, 9999999));
      }
    }

    // Only good updates reach this point, so use them for updating the robot pose
    posePublisher.set(limelightMeasurement.pose);
    estTimeStamp.set(limelightMeasurement.timestampSeconds);
    m_swerveDriveTrain.addVisionMeasurement(
        limelightMeasurement.pose, limelightMeasurement.timestampSeconds);

    // Reset the Swerve Pose with MegaTag1 if we are disabled
    if (DriverStation.isDisabled() && !limelightMeasurement.isMegaTag2) {
      m_swerveDriveTrain.resetPose(limelightMeasurement.pose);
    }

    return true;
  }

  @Logged(name = "Has Initial Pose", importance = Logged.Importance.INFO)
  public boolean getInitialPose() {
    return this.hasInitialPose;
  }

  /** Stop the nearest target from updating when we want to score to avoid target switching */
  public void setTargetLock(boolean set) {
    lockTarget = set;
  }

  @Logged(name = "On Target", importance = Logged.Importance.CRITICAL)
  public boolean isOnTarget() {
    var translationDelta = getTranslationDelta();
    SmartDashboard.putNumber("Target Translation Delta", translationDelta.in(Degrees));

    return translationDelta.lt(Degrees.of(1));
  }

  public Angle getTranslationDelta() {
    return m_swerveDriveTrain
                    .getState()
                    .Pose
                    .getRotation()
                    .minus(robotToTarget[1].getRotation())
                    .getMeasure();
  }

  @Override
  public void periodic() {
    // limelight r
    boolean llaRSuccess = processLimelight(CAMERA_SERVER.limelightR.toString(), estPoseLLR);
    SmartDashboard.putBoolean(CAMERA_SERVER.limelightR + " UpdatedRejected", llaRSuccess);

    // limelight l
    boolean llaLSuccess = processLimelight(CAMERA_SERVER.limelightL.toString(), estPoseLLL);
    SmartDashboard.putBoolean(CAMERA_SERVER.limelightL + " UpdatedRejected", llaLSuccess);

    if (!m_localized) {
      // TODO: Change this to check if the robotPose and both limelight are all close to each other
      m_localized = llaRSuccess && llaLSuccess;
    }

    // if (m_swerveDriveTrain != null) {
    //   updateAngleToHub();
    // }
  }

  @Override
  public void simulationPeriodic() {
    // if (m_swerveDriveTrain != null) {
    // visionSim.update(m_swerveDriveTrain.getState().Pose);
    // visionSim.getDebugField().setRobotPose(m_swerveDriveTrain.getState().Pose);
    // }
  }
}
