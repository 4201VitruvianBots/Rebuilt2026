package frc.team4201.lib.geometry;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.*;

public class Targeting2d {
  private static AprilTagFieldLayout wpilibAprilTagLayout;
  private static final Map<String, Target2d> m_targetList = new HashMap<>();
  private static final Map<Integer, AprilTagData> m_aprilTagList = new HashMap<>();

  public static void init() {
    if (wpilibAprilTagLayout == null) {
      wpilibAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }
    var tags = wpilibAprilTagLayout.getTags();
    for (var t : tags) {
      var tagData = new AprilTagData(t.ID, t.pose.toPose2d());
      m_aprilTagList.put(t.ID, tagData);
    }
  }

  public static void addTarget2d(String targetName, Pose2d targetPose, int... aprilTagIds) {
    var target = new Target2d(targetName, targetPose);
    for (var id : aprilTagIds) {
      target.addAprilTag(id);
      m_aprilTagList.get(id).addRelativeTarget(target);
    }
  }

  public static Optional<Pose2d> getTarget(String name) {
    return getTarget(name, -1);
  }

  public static Optional<Pose2d> getTarget(String name, int nearestTagId) {
    return Optional.ofNullable(m_aprilTagList.get(nearestTagId))
        .map(data -> data.getRelativeTargetPose(name))
        .orElse(Optional.ofNullable(m_targetList.get(name).getPose()));
  }

  static class Target2d {
    private final String m_name;
    private final Pose2d m_pose;
    private final ArrayList<Integer> m_associatedAprilTags = new ArrayList<>();

    public Target2d(String name, Pose2d pose) {
      m_name = name;
      m_pose = pose;
    }

    public void addAprilTag(int aprilTagId) {
      m_associatedAprilTags.add(aprilTagId);
    }

    public String getName() {
      return m_name;
    }

    public Pose2d getPose() {
      return m_pose;
    }

    public ArrayList<Integer> getAssociatedAprilTags() {
      return m_associatedAprilTags;
    }
  }

  static class AprilTagData {
    private final int m_tagId;
    private final Pose2d m_tagPose;
    private final Map<String, Translation2d> m_relativeTargetTranslations = new HashMap<>();
    private final Map<String, Pose2d> m_relativeTargetPoses = new HashMap<>();

    public AprilTagData(int id, Pose2d tagPose) {
      m_tagId = id;
      m_tagPose = tagPose;
    }

    public int getId() {
      return m_tagId;
    }

    public Pose2d getPose() {
      return m_tagPose;
    }

    public void addRelativeTarget(Target2d target) {
      var transform = target.getPose().minus(getPose());
      m_relativeTargetTranslations.put(target.getName(), transform.getTranslation());
      m_relativeTargetPoses.put(target.getName(), getPose().plus(transform));
    }

    public Optional<Translation2d> getRelativeTargetTranslation(String targetName) {
      return Optional.ofNullable(m_relativeTargetTranslations.get(targetName));
    }

    public Optional<Pose2d> getRelativeTargetPose(String targetName) {
      return Optional.ofNullable(m_relativeTargetPoses.get(targetName));
    }
  }
}
