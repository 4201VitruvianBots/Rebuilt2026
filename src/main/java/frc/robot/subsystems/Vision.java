package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FIELD;
import frc.robot.constants.VISION;
import frc.robot.constants.VISION.CAMERA_SERVER;
import frc.robot.constants.VISION.TARGET;
import frc.team4201.lib.simulation.FieldSim;
import frc.team4201.lib.utils.ConcurrentTimeInterpolatableBuffer;
import frc.team4201.lib.vision.LimelightHelpers;
import java.util.Optional;

public class Vision extends SubsystemBase {
  private CommandSwerveDrivetrain m_swerveDriveTrain;
  private FieldSim m_fieldSim;
  private Translation2d m_goal = FIELD.HUB.GOAL.getTargetPosition().toTranslation2d();
  // TODO: Re-add this
  //   private LimelightSim visionSim;
  private Controls m_controls;

  VISION.Limelight LLL = new VISION.Limelight(CAMERA_SERVER.limelightL);
  VISION.Limelight LLR = new VISION.Limelight(CAMERA_SERVER.limelightR);

  private boolean m_localized;

  private TARGET m_currentTarget = TARGET.LEFT_FRONT_TOWER;
  private Pose2d targetPose = Pose2d.kZero;

  private boolean lockTarget = false;
  private boolean hasInitialPose = false;

  // NetworkTables publisher setup
  public final DoubleSubscriber m_kPAutoAlignSubscriber;
  public final DoubleSubscriber m_kDAutoAlignSubscriber;

  public final DoublePublisher m_kPAutoAlignPublisher;
  public final DoublePublisher m_kDAutoAlignPublisher;

  public Vision(Controls controls) {
    m_controls = controls;

    var topickP =
        NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("kPAutoAlign");
    var topickD =
        NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("kDAutoAlign");
    m_kPAutoAlignSubscriber = topickP.subscribe(7.4);
    m_kDAutoAlignSubscriber = topickD.subscribe(0.3);

    m_kPAutoAlignPublisher = topickP.publish();
    m_kDAutoAlignPublisher = topickD.publish();

    setName("Vision");
  }

  public void registerSwerveDrive(CommandSwerveDrivetrain swerveDriveTrain) {
    m_swerveDriveTrain = swerveDriveTrain;
  }

  public void registerFieldSim(FieldSim fieldSim) {
    m_fieldSim = fieldSim;
  }

  @Logged(name = "Left Target", importance = Logged.Importance.CRITICAL)
  public boolean isTargetingLeft() {
    return m_currentTarget
        == TARGET.LEFT_FRONT_TOWER /* || m_currentTarget == TARGET.LEFT_BACK_TOWER */;
  }

  @Logged(name = "Right Target", importance = Logged.Importance.CRITICAL)
  public boolean isTargetingRight() {
    return m_currentTarget
        == TARGET.RIGHT_FRONT_TOWER /* || m_currentTarget == TARGET.RIGHT_BACK_TOWER */;
  }

  public Pose2d updateClimbTarget(TARGET target) {
    if (lockTarget) return targetPose;
    targetPose = m_swerveDriveTrain.getState().Pose;
    switch (target) {
      case LEFT_FRONT_TOWER:
        if (Controls.isBlueAlliance()) {
          targetPose =
              new Pose2d(
                  FIELD.TOWER.BLUE.LEFT.getTargetPosition().getMeasureX(),
                  FIELD.TOWER.BLUE.LEFT.getTargetPosition().getMeasureY(),
                  Rotation2d.kZero);
        } else {
          targetPose =
              new Pose2d(
                  FIELD.TOWER.RED.LEFT.getTargetPosition().getMeasureX(),
                  FIELD.TOWER.RED.LEFT.getTargetPosition().getMeasureY(),
                  Rotation2d.k180deg);
        }
        break;
      case RIGHT_FRONT_TOWER:
        if (Controls.isBlueAlliance()) {
          targetPose =
              new Pose2d(
                  FIELD.TOWER.BLUE.RIGHT.getTargetPosition().getMeasureX(),
                  FIELD.TOWER.BLUE.RIGHT.getTargetPosition().getMeasureY(),
                  Rotation2d.kZero);
        } else {
          targetPose =
              new Pose2d(
                  FIELD.TOWER.RED.RIGHT.getTargetPosition().getMeasureX(),
                  FIELD.TOWER.RED.RIGHT.getTargetPosition().getMeasureY(),
                  Rotation2d.k180deg);
        }
        break;
      default:
        break;
    }
    return targetPose;
  }

  final double LOOKBACK_TIME = 1.0;
  private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
      ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME);

  public class VisionFieldPoseEstimate {

    private final Pose2d visionRobotPoseMeters;
    private final double timestampSeconds;
    private final Matrix<N3, N1> visionMeasurementStdDevs;
    private final int numTags;

    public VisionFieldPoseEstimate(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs,
        int numTagsFuse) {
      this.visionRobotPoseMeters = visionRobotPoseMeters;
      this.timestampSeconds = timestampSeconds;
      this.visionMeasurementStdDevs = visionMeasurementStdDevs;
      this.numTags = numTagsFuse;
    }

    public Pose2d getVisionRobotPoseMeters() {
      return visionRobotPoseMeters;
    }

    public double getTimestampSeconds() {
      return timestampSeconds;
    }

    public Matrix<N3, N1> getVisionMeasurementStdDevs() {
      return visionMeasurementStdDevs;
    }

    public int getNumTags() {
      return numTags;
    }
  }

  public Optional<Pose2d> getFieldToRobot(double timestamp) {
    return fieldToRobot.getSample(timestamp);
  }

  private VisionFieldPoseEstimate fuseEstimates(
      VisionFieldPoseEstimate lla, VisionFieldPoseEstimate llb) {
    if (llb.getTimestampSeconds() < lla.getTimestampSeconds()) {
      VisionFieldPoseEstimate lltmp = lla;
      lla = llb;
      llb = lltmp;
    }

    // TODO: find out what this is and why they need it

    Transform2d a_T_b =
        getFieldToRobot(llb.getTimestampSeconds())
            .get()
            .minus(getFieldToRobot(lla.getTimestampSeconds()).get());

    Pose2d poseA = lla.getVisionRobotPoseMeters().transformBy(a_T_b);
    Pose2d poseB = llb.getVisionRobotPoseMeters();

    // inverse variance weighting
    Matrix<N3, N1> varianceA =
        lla.getVisionMeasurementStdDevs().elementTimes(lla.getVisionMeasurementStdDevs());
    Matrix<N3, N1> varianceB =
        llb.getVisionMeasurementStdDevs().elementTimes(llb.getVisionMeasurementStdDevs());

    Rotation2d fusedHeading = poseB.getRotation();

    final double kLargeVariance = 1e6;
    if (varianceA.get(2, 0) < kLargeVariance && varianceB.get(2, 0) < kLargeVariance) {
      fusedHeading =
          new Rotation2d(
              poseA.getRotation().getCos() / varianceA.get(2, 0)
                  + poseB.getRotation().getCos() / varianceB.get(2, 0),
              poseA.getRotation().getSin() / varianceA.get(2, 0)
                  + poseB.getRotation().getSin() / varianceB.get(2, 0));
    }

    double weightAx = 1.0 / varianceA.get(0, 0);
    double weightAy = 1.0 / varianceA.get(1, 0);
    double weightBx = 1.0 / varianceB.get(0, 0);
    double weightBy = 1.0 / varianceB.get(1, 0);

    // double headingVarA = Math.max(varianceA.get(2,0), 1e-4);
    // double headingVarB = Math.max(varianceB.get(2,0), 1e-4);
    // double weightA = 1.0 / headingVarA;
    // double weightB = 1.0 / headingVarB;

    Pose2d fusedPose =
        new Pose2d(
            new Translation2d(
                (poseA.getTranslation().getX() * weightAx
                        + poseB.getTranslation().getX() * weightBx)
                    / (weightAx + weightBx),
                (poseA.getTranslation().getY() * weightAy
                    + poseB.getTranslation().getY() * weightBy)),
            fusedHeading);

    Matrix<N3, N1> fusedStdDev =
        VecBuilder.fill(
            Math.sqrt(1.0 / (weightAx + weightBx)),
            Math.sqrt(1.0 / (weightAy + weightBy)),
            Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

    int numTags = lla.getNumTags() + llb.getNumTags();
    double time = llb.getTimestampSeconds();

    return new VisionFieldPoseEstimate(fusedPose, time, fusedStdDev, numTags);
  }

  /**
   * Process measurements from a limelight. Return true if the given vision measurement is used,
   * otherwise return false to indicate that it was rejected.
   */
  public boolean processLimelight(VISION.Limelight limelight) {
    String limelightName = limelight.getName();
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

    boolean validResult = isPoseValid(limelight, limelightMeasurement);

    // Log Data
    limelight.getHeartbeat();
    limelight.publishValid(validResult);
    if (limelightMeasurement != null) {
      limelight.publishTimestamp(limelightMeasurement.timestampSeconds);
      limelight.publishPose(limelightMeasurement.pose);
      limelight.publishTagCount(limelightMeasurement.tagCount);
      limelight.publishMegatag2Pose(limelightMeasurement.isMegaTag2);
    } else {
      limelight.publishTimestamp(-1);
      limelight.publishPose(new Pose2d(-1, -1, Rotation2d.kZero));
      limelight.publishTagCount(-1);
      limelight.publishMegatag2Pose(false);
    }

    if (validResult) {
      limelight.setLastGoodEstimate(limelightMeasurement);
    }

    return validResult;
  }

  public boolean isPoseValid(
      VISION.Limelight limelight, LimelightHelpers.PoseEstimate poseEstimate) {
    if (poseEstimate == null) {
      if (RobotBase.isReal())
        DriverStation.reportWarning(limelight.getName() + " is not connected", true);
      return false;
    } else {
      // Filter out bad AprilTag vision estimates for both MegaTag1 and MegaTag2
      if (poseEstimate.timestampSeconds == 0) {
        return false;
      } else if (poseEstimate.pose.getTranslation().equals(Translation2d.kZero)) {
        return false;
      } else if (poseEstimate.tagCount == 0) {
        return false;
      }

      if (!poseEstimate.isMegaTag2) {
        // Filter out bad AprilTag vision estimates for MegaTag1
        // TODO: Check 1 tag from center?
        if (poseEstimate.tagCount < 2) {
          return false;
        }

        hasInitialPose = true;
      } else {
        // Ignore MegaTag2 updates if the robot is spinning too fast
        if (m_swerveDriveTrain.getGyroYawRate().abs(DegreesPerSecond) > 720.0) {
          return false;
        }
      }

      // Set Standard Deviations
      m_swerveDriveTrain.setVisionMeasurementStdDevs(limelight.getStdDev(poseEstimate.isMegaTag2));
    }

    return true;
  }

  @Logged(name = "Has Initial Pose", importance = Logged.Importance.INFO)
  public boolean getInitialPose() {
    return this.hasInitialPose;
  }

  @Logged(name = "LLL Connected", importance = Logged.Importance.INFO)
  public boolean lllConnected() {
    return LLL.isAlive();
  }

  @Logged(name = "LLR Connected", importance = Logged.Importance.INFO)
  public boolean llrConnected() {
    return LLR.isAlive();
  }

  /** Stop the nearest target from updating when we want to score to avoid target switching */
  public void setTargetLock(boolean set) {
    lockTarget = set;
  }

  public Rotation2d getAngleToTarget() {
    return m_swerveDriveTrain
        .getState()
        .Pose
        .getTranslation()
        .minus(targetPose.getTranslation())
        .getAngle()
        .plus(m_swerveDriveTrain.getState().Pose.getRotation());

    // var setPoint = m_goal.minus(m_swerveDriveTrain.getState().Pose.getTranslation());
    // SmartDashboard.putBoolean("Aligned to Hub?", isAligned);
    // System.out.println("The angle to the hub is " + setPoint.getAngle());
    // System.out.println("The robot's angle is " +
    // m_swerveDriveTrain.getState().Pose.getRotation());
    // System.out.println("Therefore, the alignment is" + isAligned);
  }

  @Logged(name = "On Target", importance = Logged.Importance.DEBUG)
  public boolean isOnTarget() {
    return getAngleToTarget().getDegrees() < 0.5;
  }

  public boolean isPointingAtGoal(
      Translation2d goal, double tolerance, boolean returnAbsoluteValue) {
    // bearing from robot to goal
    var bearing =
        goal.minus(m_swerveDriveTrain.getState().Pose.getTranslation()).getAngle().getRadians();
    // robot heading
    var heading = m_swerveDriveTrain.getState().Pose.getRotation().getRadians();
    // smallest signed angle difference in [-pi, pi]
    double error = Math.atan2(Math.sin(bearing - heading), Math.cos(bearing - heading));
    if (returnAbsoluteValue) {
      return Math.abs(error) <= Units.degreesToRadians(tolerance);
    } else {
      return error <= Units.degreesToRadians(tolerance);
    }
  }

  @Logged(name = "Is in Neutral Sector?", importance = Importance.DEBUG)
  public boolean isInNeutralSector() {
    return FIELD.getCurrentSector().name().startsWith("NEUTRAL");
  }

  @Logged(name = "Is in Red Sector?", importance = Importance.DEBUG)
  public boolean isInRedSector() {
    return FIELD.getCurrentSector().name().startsWith("RED");
  }

  @Logged(name = "Is in Blue Sector?", importance = Importance.DEBUG)
  public boolean isInBlueSector() {
    return FIELD.getCurrentSector().name().startsWith("BLUE");
  }

  @Logged(name = "Is in Opposing Alliance Sector?", importance = Importance.DEBUG)
  public boolean isInOpposingAllianceSector() {
    if (Controls.isBlueAlliance()) {
      return isInRedSector();
    } else {
      return isInBlueSector();
    }
  }

  @Logged(name = "Is in Right Half?", importance = Importance.DEBUG)
  public boolean isInRightHalf() {
    return FIELD.getCurrentSector().name().endsWith("RIGHT");
  }

  @Logged(name = "Is in Left Half?", importance = Importance.DEBUG)
  public boolean isInLeftHalf() {
    return FIELD.getCurrentSector().name().endsWith("LEFT");
  }

  public void testInit() {
    m_kPAutoAlignPublisher.set(12.0);
    m_kDAutoAlignPublisher.set(0.0);
  }

  public void disabledPeriodic() {
    m_goal = FIELD.HUB.GOAL.getTargetPosition().toTranslation2d();
  }

  @Logged(name = "Distance to Hub", importance = Importance.INFO)
  public Distance getDistanceToHub() {
    return Meters.of(m_swerveDriveTrain.getState().Pose.getTranslation().getDistance(m_goal));
  }

  @Override
  public void periodic() {
    // limelight-left
    boolean lllSuccess = processLimelight(LLL);
    // limelight-right
    boolean llrSuccess = processLimelight(LLR);

    if (DriverStation.isDisabled()) {
      if (lllSuccess) {
        m_swerveDriveTrain.resetGyro(LLL.getLastGoodEstimate().pose.getRotation().getDegrees());
      } else if (llrSuccess) {
        m_swerveDriveTrain.resetGyro(LLR.getLastGoodEstimate().pose.getRotation().getDegrees());
      }
    }

    if (!m_localized) {
      // TODO: Change this to check if the robotPose and both limelight are all close to each other
      m_localized = lllSuccess && llrSuccess;
    }

    // Only good updates reach this point, so use them for updating the robot pose
    if (lllSuccess && llrSuccess) {
      fuseEstimates(
          new VisionFieldPoseEstimate(
              LLL.getLastGoodEstimate().pose,
              LLL.getLastGoodEstimate().timestampSeconds,
              LLL.getStdDev(),
              LLL.getLastGoodEstimate().tagCount),
          new VisionFieldPoseEstimate(
              LLR.getLastGoodEstimate().pose,
              LLR.getLastGoodEstimate().timestampSeconds,
              LLR.getStdDev(),
              LLR.getLastGoodEstimate().tagCount));
    } else if (lllSuccess) {
      m_swerveDriveTrain.addVisionMeasurement(LLL);
    } else if (llrSuccess) {
      m_swerveDriveTrain.addVisionMeasurement(LLR);
    }
  }

  @Override
  public void simulationPeriodic() {
    // if (m_swerveDriveTrain != null) {
    // visionSim.update(m_swerveDriveTrain.getState().Pose);
    // visionSim.getDebugField().setRobotPose(m_swerveDriveTrain.getState().Pose);
    // }
  }
}
