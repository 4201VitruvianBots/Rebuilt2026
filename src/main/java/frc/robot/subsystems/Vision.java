package frc.robot.subsystems;

import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Meters;

import com.limelightvision.*;
import com.limelightvision.PoseEstimate;
import frc.robot.constants.VISION;
import org.wpilib.command2.SubsystemBase;

import com.ctre.phoenix6.Utils;

import frc.robot.constants.FIELD;
import frc.robot.constants.FIELD.BUMP_ALIGNMENT_TARGETS;
import frc.robot.constants.VISION.CAMERA_SERVER;
import frc.robot.constants.VISION.TARGET;
import frc.robot.lib.BLine.Path;
import frc.team4201.lib.simulation.FieldSim;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.RobotState;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Logged.Importance;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.units.measure.Distance;

public class Vision extends SubsystemBase {
  private CommandSwerveDrivetrain m_swerveDriveTrain;
  private FieldSim m_fieldSim;

  private Translation2d m_goal = FIELD.HUB.GOAL.getTargetPosition().toTranslation2d();
  // TODO: Re-add this
  //   private LimelightSim visionSim;
  private Controls m_controls;

  Limelight LLF = new Limelight(CAMERA_SERVER.limelightF.toString());
  VISION.LimelightTable LLF_Table = new VISION.LimelightTable(CAMERA_SERVER.limelightF);

  private boolean m_localized;

  private TARGET m_currentTarget = TARGET.LEFT_FRONT_TOWER;
  private Pose2d targetPose = Pose2d.ZERO;
  private Pose2d allianceZonePose = Pose2d.ZERO;
  private Pose2d neutralZonePose = Pose2d.ZERO;
  private Pose2d unrealisticPose = Pose2d.ZERO;

  private boolean lockTarget = false;
  private boolean hasInitialPose = false;
  private boolean matchStarted = false;

  // NetworkTables publisher setup
  public final DoubleSubscriber m_kPAutoAlignSubscriber;
  public final DoubleSubscriber m_kDAutoAlignSubscriber;

  public final DoublePublisher m_kPAutoAlignPublisher;
  public final DoublePublisher m_kDAutoAlignPublisher;

  private int[] excludeTrenchTags = {
    2, 3, 4, 5, 8, 9, 10, 11, 13, 14, 15, 16, 18, 19, 20, 21, 24, 25, 26, 27, 29, 30, 31, 32
  };

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

  public Path updateCrossBumpPath(boolean endsShootingPosition) {
    if (isInLeftHalf()) {
      neutralZonePose = BUMP_ALIGNMENT_TARGETS.LEFT_NEUTRAL_BUMP.getAlignmentPose();
      if (endsShootingPosition) {
        allianceZonePose = BUMP_ALIGNMENT_TARGETS.LEFT_ALLIANCE_SHOOTING.getAlignmentPose();
      } else {
        allianceZonePose = BUMP_ALIGNMENT_TARGETS.LEFT_ALLIANCE_BUMP.getAlignmentPose();
      }
      unrealisticPose = BUMP_ALIGNMENT_TARGETS.LEFT_UNREALISTIC_POSE.getAlignmentPose();
    } else {
      neutralZonePose = BUMP_ALIGNMENT_TARGETS.RIGHT_NEUTRAL_BUMP.getAlignmentPose();
      if (endsShootingPosition) {
        allianceZonePose = BUMP_ALIGNMENT_TARGETS.RIGHT_ALLIANCE_SHOOTING.getAlignmentPose();
      } else {
        allianceZonePose = BUMP_ALIGNMENT_TARGETS.RIGHT_ALLIANCE_BUMP.getAlignmentPose();
      }
      unrealisticPose = BUMP_ALIGNMENT_TARGETS.RIGHT_UNREALISTIC_POSE.getAlignmentPose();
    }

    return new Path(
        new Path.Waypoint(neutralZonePose, 0.8),
        new Path.Waypoint(unrealisticPose, 4.2),
        new Path.Waypoint(allianceZonePose, 1.0));
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
                  Rotation2d.ZERO);
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
                  Rotation2d.ZERO);
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

  /**
   * Process measurements from a limelight. Return true if the given vision measurement is used,
   * otherwise return false to indicate that it was rejected.
   */
  public boolean processLimelight(Limelight limelight) {
    String limelightName = limelight.getName();
    if (RobotState.isDisabled()) {
      // TODO: Determine if we change IMUMode to 0 when not disabled for MegaTag2
      limelight.setIMUMode(IMUMode.EXTERNAL_SEED_INTERNAL);
    }
    limelight.setRobotOrientation(
        m_swerveDriveTrain.getState().Pose.getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0,
        true);

    PoseEstimate limelightMeasurement;
    if (RobotState.isDisabled() || m_swerveDriveTrain.getState().Velocity.omega <= 0.1) {
      // Use MegaTag1 when the robot is disabled to set the initial robot pose

      limelightMeasurement = limelight.getPoseEstimate(PoseEstimateType.MT1_WPIBLUE);
    } else {
      // Use MegaTag2 when the robot is running for more accurate pose updates
      limelightMeasurement = limelight.getPoseEstimate(PoseEstimateType.MT2_WPIBLUE);
    }

    boolean validResult = isPoseValid(limelight, limelightMeasurement);

    // Log Data
    LLF_Table.publishValid(validResult);
    if (limelightMeasurement.isValid() && validResult) {
      LLF_Table.publishTimestamp(limelightMeasurement.timestampSeconds);
      LLF_Table.publishRobotTimestamp(Utils.getCurrentTimeSeconds());
      LLF_Table.publishPose(limelightMeasurement.pose);
      LLF_Table.publishTagCount(limelightMeasurement.reportedTagCount);
      LLF_Table.publishMegatag2Pose(limelightMeasurement.isMT2());
    } else {
      LLF_Table.publishTimestamp(-1);
      LLF_Table.publishRobotTimestamp(-1);
      LLF_Table.publishPose(new Pose2d(-1, -1, Rotation2d.ZERO));
      LLF_Table.publishTagCount(-1);
      LLF_Table.publishMegatag2Pose(false);
    }

    if (validResult) {
      // Only good updates reach this point, so use them for updating the robot pose

        // Reset the Swerve Pose with MegaTag1 if we are disabled
      if (!limelightMeasurement.isMT2()) {
        m_swerveDriveTrain.resetPose(limelightMeasurement.pose);
      } else {
        m_swerveDriveTrain.addVisionMeasurement(
            limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
      }
    }

    return validResult;
  }

  public boolean isPoseValid(
      Limelight limelight, PoseEstimate poseEstimate) {
    if (poseEstimate == null) {
      if (RobotBase.isReal())
        DriverStationErrors.reportWarning(limelight.getName() + " is not connected", true);
      return false;
    } else {
      // Filter out bad AprilTag vision estimates for both MegaTag1 and MegaTag2
      if (poseEstimate.timestampSeconds == 0) {
        return false;
      } else if (poseEstimate.pose.getTranslation().equals(Translation2d.ZERO)) {
        return false;
      } else if (poseEstimate.reportedTagCount == 0) {
        return false;
      }

      if (!poseEstimate.isMT2()) {
        // Filter out bad AprilTag vision estimates for MegaTag1
        // TODO: Check 1 tag from center?
        if (poseEstimate.reportedTagCount < 2) {
          return false;
        }

        limelight.setFiducialIDFiltersOverride(excludeTrenchTags);

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

    return true;
  }

  @Logged(name = "Has Initial Pose", importance = Logged.Importance.INFO)
  public boolean getInitialPose() {
    return this.hasInitialPose;
  }

  @Logged(name = "LLF Connected", importance = Logged.Importance.INFO)
  public boolean llfConnected() {
    return LLF.isConnected();
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
        .orElse(Rotation2d.ZERO)
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
    if (RobotState.isAutonomous()) {
      return Math.abs(getAngleToTarget().getDegrees()) < 2.0;
    } else {
      return Math.abs(getAngleToTarget().getDegrees()) < 0.5;
    }
  }

  public boolean isPointingAtGoal(
      Translation2d goal, double tolerance, boolean returnAbsoluteValue) {
    // bearing from robot to goal
    var bearing =
        goal.minus(m_swerveDriveTrain.getState().Pose.getTranslation())
            .getAngle()
            .orElse(Rotation2d.ZERO)
            .getRadians();
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

  public void utilityInit() {
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
    // limelight-front
    boolean lllSuccess = processLimelight(LLF);

    if (!m_localized) {
      // TODO: Change this to check if the robotPose and both limelight are all close to each other
      m_localized = lllSuccess;
    }

    // Do this to avoid issues with the brief 'disabled' period between auto and teleop
    if (RobotState.isFMSAttached()
        && RobotState.isAutonomous()
        && !matchStarted) {
      matchStarted = true;
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
