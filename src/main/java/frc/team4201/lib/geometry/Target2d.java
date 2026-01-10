package frc.team4201.lib.geometry;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.BiConsumer;

public class Target2d {

  private static AprilTagFieldLayout wpilibAprilTagLayout;

  private final int m_apriltagId;
  private Pose2d m_targetPose = Pose2d.kZero;
  private Pose2d m_goalPose = Pose2d.kZero;

  public Target2d(int apriltagId) {
    this(apriltagId, (target, goal) -> goal = target);
  }

  public Target2d(int apriltagId, BiConsumer<Pose2d, Pose2d> tagToGoal) {
    if (wpilibAprilTagLayout == null) {
      wpilibAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    m_apriltagId = apriltagId;

    var tagPose = wpilibAprilTagLayout.getTagPose(m_apriltagId);
    tagPose.ifPresent(pose3d -> m_targetPose = pose3d.toPose2d());
    tagToGoal.accept(m_targetPose, m_goalPose);
  }

  public Pose2d getTarget() {
    return Pose2d.kZero;
  }

  public Pose2d getTargetGoal() {
    return Pose2d.kZero;
  }
}
