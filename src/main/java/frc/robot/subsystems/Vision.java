package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FIELD;
import frc.robot.constants.VISION.CAMERA_SERVER;
import frc.robot.constants.VISION.TARGET;
import frc.team4201.lib.simulation.FieldSim;
import frc.team4201.lib.utils.ConcurrentTimeInterpolatableBuffer;
import frc.team4201.lib.vision.LimelightHelpers;
import java.util.Optional;

public class Vision extends SubsystemBase {
  private CommandSwerveDrivetrain m_swerveDriveTrain;
  private FieldSim m_fieldSim;
  private Translation2d m_goal = new Translation2d();
  // TODO: Re-add this
  //   private LimelightSim visionSim;
  private Controls m_controls;

  private boolean m_localized;

  private TARGET m_currentTarget = TARGET.LEFT_FRONT_TOWER;
  private Pose2d targetPose = Pose2d.kZero;

  private boolean lockTarget = false;
  private boolean hasInitialPose = false;
  // NetworkTables publisher setup
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("LimelightPoseEstimate");

  public final DoubleSubscriber m_kPAutoAlignSubscriber;
  public final DoubleSubscriber m_kDAutoAlignSubscriber;

  private final DoublePublisher estTimeStamp = table.getDoubleTopic("estTimeStamp").publish();
  public final DoublePublisher m_kPAutoAlignPublisher;
  public final DoublePublisher m_kDAutoAlignPublisher;

  private final StructPublisher<Pose2d> estPoseLLR =
      table.getStructTopic("estPoseLLR", Pose2d.struct).publish();

  private final StructPublisher<Pose2d> estPoseLLL =
      table.getStructTopic("estPoseLLL", Pose2d.struct).publish();

  public Vision(Controls controls) {
    m_controls = controls;
    m_goal = FIELD.HUB.GOAL.getTargetPosition().toTranslation2d();
    registerSwerveDrive(m_swerveDriveTrain);
    // Port Forwarding to access limelight web UI on USB Ethernet
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, CAMERA_SERVER.limelightR.toString(), port);
      PortForwarder.add(port + 10, CAMERA_SERVER.limelightL.toString(), port);
    }

    var topickP =
        NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("kPAutoAlign");
    var topickD =
        NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic("kDAutoAlign");
    m_kPAutoAlignSubscriber = topickP.subscribe(7.4);
    m_kDAutoAlignSubscriber = topickD.subscribe(0.3);

    m_kPAutoAlignPublisher = topickP.publish();
    m_kDAutoAlignPublisher = topickD.publish();
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
                  new Rotation2d());
        } else {
          targetPose =
              new Pose2d(
                  FIELD.TOWER.RED.LEFT.getTargetPosition().getMeasureX(),
                  FIELD.TOWER.RED.LEFT.getTargetPosition().getMeasureY(),
                  new Rotation2d(Degrees.of(180)));
        }
        break;
      case RIGHT_FRONT_TOWER:
        if (Controls.isBlueAlliance()) {
          targetPose =
              new Pose2d(
                  FIELD.TOWER.BLUE.RIGHT.getTargetPosition().getMeasureX(),
                  FIELD.TOWER.BLUE.RIGHT.getTargetPosition().getMeasureY(),
                  new Rotation2d());
        } else {
          targetPose =
              new Pose2d(
                  FIELD.TOWER.RED.RIGHT.getTargetPosition().getMeasureX(),
                  FIELD.TOWER.RED.RIGHT.getTargetPosition().getMeasureY(),
                  new Rotation2d(Degrees.of(180)));
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

    // public final VisionFieldPoseEstimate lla;
    // public final VisionFieldPoseEstimate llb;

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
  public Optional<VisionFieldPoseEstimate> processLimelight(
      String limelightName, StructPublisher<Pose2d> posePublisher) {
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

    //    var llResults = LimelightHelpers.getLatestResults(limelightName);
    LimelightHelpers.PoseEstimate limelightMeasurement;
    if (DriverStation.isDisabled()) {
      // Use MegaTag1 when the robot is disabled to set the initial robot pose
      limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    } else {
      // Use MegaTag2 when the robot is running for more accurate pose updates
      limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    Vector<N3> standardDeviations;
    if (limelightMeasurement == null) {
      if (RobotBase.isReal())
        DriverStation.reportWarning(limelightName + " is not connected", true);
      return Optional.empty();
    } else {
      // Filter out bad AprilTag vision estimates for both MegaTag1 and MegaTag2
      if (limelightMeasurement.timestampSeconds == 0) {
        return Optional.empty();
      } else if (limelightMeasurement.pose.getTranslation().equals(Translation2d.kZero)) {
        return Optional.empty();
      } else if (limelightMeasurement.tagCount == 0) {
        return Optional.empty();
      }

      if (!limelightMeasurement.isMegaTag2) {
        // Filter out bad AprilTag vision estimates for MegaTag1
        // TODO: Check 1 tag from center?
        if (limelightMeasurement.tagCount < 1) {
          return Optional.empty();
        }

        hasInitialPose = true;
        // Set Standard Deviations for MegaTag1
        standardDeviations = VecBuilder.fill(.5, .5, 9999999);
      } else {
        // Ignore MegaTag2 updates if the robot is spinning too fast
        if (m_swerveDriveTrain.getGyroYawRate().abs(DegreesPerSecond) > 720.0) {
          return Optional.empty();
        }

        // Set Standard Deviations for MegaTag2
        standardDeviations = VecBuilder.fill(.4, .4, 9999999);
      }
    }

    // Only good updates reach this point, so use them for updating the robot pose
    posePublisher.set(limelightMeasurement.pose);
    estTimeStamp.set(limelightMeasurement.timestampSeconds);

    // TODO: Figure out how 254 calculates this/gets it from the limelight results
    //    int[] fiducialIds = new int[limelightMeasurement.rawFiducials.length];
    //    for (int i = 0; i < limelightMeasurement.rawFiducials.length; i++) {
    //      if (limelightMeasurement.rawFiducials[i] != null) {
    //        fiducialIds[i] = limelightMeasurement.rawFiducials[i].id;
    //      }
    //    }
    //
    //    var quality = fiducialIds.length > 1 ? 1.0 : 1.0 -
    // limelightMeasurement.rawFiducials[0].ambiguity;
    //
    //    double scaleFactor = 1.0 / quality;
    //    double xStd = cam.standardDeviations[VISION.kMegatag1XStdDevIndex] * scaleFactor;
    //    double yStd = cam.standardDeviations[VISION.kMegatag1YStdDevIndex] * scaleFactor;
    //    double rotStd = cam.standardDeviations[VISION.kMegatag1YawStdDevIndex] * scaleFactor;
    //    var standardDeviations = VecBuilder.fill(xStd, yStd, rotStd);

    return Optional.of(
        new VisionFieldPoseEstimate(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds,
            standardDeviations, // TODO: How to get this value
            limelightMeasurement.tagCount));
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
    var rotationDelta =
        m_swerveDriveTrain
            .getState()
            .Pose
            .getTranslation()
            .minus(targetPose.getTranslation())
            .getAngle()
            .plus(m_swerveDriveTrain.getState().Pose.getRotation());

    var isAligned = rotationDelta.getDegrees() < 0.5;

    // var setPoint = m_goal.minus(m_swerveDriveTrain.getState().Pose.getTranslation());
    // SmartDashboard.putBoolean("Aligned to Hub?", isAligned);
    // System.out.println("The angle to the hub is " + setPoint.getAngle());
    // System.out.println("The robot's angle is " +
    // m_swerveDriveTrain.getState().Pose.getRotation());
    // System.out.println("Therefore, the alignment is" + isAligned);

    return isAligned;
  }

  @Logged(name = "Is Pointing at Goal", importance = Importance.INFO)
  public boolean isPointingAtGoal() {
    // bearing from robot to goal
    var bearing =
        m_goal.minus(m_swerveDriveTrain.getState().Pose.getTranslation()).getAngle().getRadians();
    // robot heading
    var heading = m_swerveDriveTrain.getState().Pose.getRotation().getRadians();
    // smallest signed angle difference in [-pi, pi]
    double error = Math.atan2(Math.sin(bearing - heading), Math.cos(bearing - heading));
    return Math.abs(error) <= Units.degreesToRadians(1.0);
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
    m_kPAutoAlignPublisher.set(7.4);
    m_kDAutoAlignPublisher.set(0.3);
  }

  public void teleopInit() {}

  @Logged(name = "Distance to Hub", importance = Importance.INFO)
  public Distance getDistanceToHub() {
    return Meters.of(m_swerveDriveTrain.getState().Pose.getTranslation().getDistance(m_goal));
  }

  @Override
  public void periodic() {
    // limelight r
    var llRPose = processLimelight(CAMERA_SERVER.limelightR.toString(), estPoseLLR);
    SmartDashboard.putBoolean(CAMERA_SERVER.limelightR + " UpdatedRejected", llRPose.isPresent());

    // limelight l
    var lllPose = processLimelight(CAMERA_SERVER.limelightL.toString(), estPoseLLL);
    SmartDashboard.putBoolean(CAMERA_SERVER.limelightL + " UpdatedRejected", lllPose.isPresent());

    if (llRPose.isPresent() && lllPose.isPresent()) {
      var fusedPose = fuseEstimates(lllPose.get(), llRPose.get());
      if (DriverStation.isDisabled()) {
        m_swerveDriveTrain.resetPose(fusedPose.visionRobotPoseMeters);
      } else {
        m_swerveDriveTrain.addVisionMeasurement(
            fusedPose.visionRobotPoseMeters,
            fusedPose.timestampSeconds,
            fusedPose.visionMeasurementStdDevs);
      }

      if (!m_localized) {
        // TODO: Change this to check if the robotPose and both limelight are all close to each
        // other
        m_localized = true;
      }
    }

    // if (m_swerveDriveTrain != null) {
    //   updateAngleToHub();
    // }

    isOnTarget();
  }

  @Override
  public void simulationPeriodic() {
    // if (m_swerveDriveTrain != null) {
    // visionSim.update(m_swerveDriveTrain.getState().Pose);
    // visionSim.getDebugField().setRobotPose(m_swerveDriveTrain.getState().Pose);
    // }
  }
}
