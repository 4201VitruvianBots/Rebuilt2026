package frc.team4201.lib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

public class Limelight {
  private final String m_name;
  private final String m_ip;

  private final NetworkTableInstance nt_instance = NetworkTableInstance.getDefault();
  private final NetworkTable nt_limelights = nt_instance.getTable("Limelights");
  private final NetworkTable nt_limelight_instance;
  private final StructPublisher<Pose2d> m_posePublisher;

  private Pose2d m_lastValidPose = Pose2d.kZero;
  private Optional<LimelightHelpers.PoseEstimate> m_lastValidMeasurement = Optional.empty();
  private boolean m_initialPoseSet = false;

  private static final Matrix<N3, N1> megaTag1Std = VecBuilder.fill(.5, .5, 9999999);
  private static final Matrix<N3, N1> megaTag2Std = VecBuilder.fill(.4, .4, 9999999);
  private Matrix<N3, N1> currentStd = megaTag2Std;

  public Limelight(String name, String ip) {
    m_name = name;
    m_ip = ip;

    nt_limelight_instance = nt_limelights.getSubTable(m_name);
    m_posePublisher =
        nt_limelight_instance.getStructTopic("Estimated Pose", Pose2d.struct).publish();

    // LimelightHelpers.setCameraPose_RobotSpace(m_name);

  }

  public Optional<LimelightHelpers.PoseEstimate> getLastValidMeasurement() {
    return m_lastValidMeasurement;
  }

  public Pose2d getLastValidPose() {
    return m_lastValidPose;
  }

  public boolean process() {
    m_lastValidMeasurement = Optional.empty();
    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode(m_name, IMU_MODE.FUSED.ordinal());
      LimelightHelpers.SetFiducialIDFiltersOverride(m_name, new int[] {});

      // Use MegaTag1 when the robot is disabled to set the initial robot pose
      m_lastValidMeasurement = Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name));
      currentStd = megaTag1Std;
    } else {
      // Use MegaTag2 when the robot is running for more accurate pose updates
      m_lastValidMeasurement =
          Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name));
      currentStd = megaTag2Std;
    }

    m_lastValidMeasurement.ifPresent(
        (measurement) -> {
          if (measurement.timestampSeconds == 0
              || measurement.pose.getTranslation().equals(Translation2d.kZero)
              || measurement.tagCount == 0) {
            return;
          }

          if (measurement.isMegaTag2 && (measurement.tagCount < 2)) {
            return;
          }

          m_initialPoseSet = true;
          m_posePublisher.set(measurement.pose);
        });

    if (m_lastValidMeasurement.isEmpty() && !m_initialPoseSet) {
      if (RobotBase.isReal()) {
        DriverStation.reportWarning("[Limelight] " + m_name + " is not connected", true);
      }
    }

    return m_lastValidMeasurement.isPresent();
  }

  private enum IMU_MODE {
    EXTERNAL,
    FUSED,
    INTERNAL,
    UNKNOWN
  }
}
