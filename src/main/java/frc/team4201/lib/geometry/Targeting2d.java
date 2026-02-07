package frc.team4201.lib.geometry;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.*;

public class Targeting2d {
  private static AprilTagFieldLayout aprilTagLayout;
  private static Translation2d fieldCenter;
  private static final Map<String, Target2d> m_targetList = new HashMap<>();
  private static final Map<Integer, AprilTagData> m_aprilTagList = new HashMap<>();

  public static void init() {
    init(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark));
  }

  public static void init(AprilTagFieldLayout layout) {
    aprilTagLayout = layout;

    fieldCenter =
        new Translation2d(
            aprilTagLayout.getFieldLength() / 2.0, aprilTagLayout.getFieldWidth() / 2.0);

    var tags = layout.getTags();
    for (var t : tags) {
      var tagData = new AprilTagData(t.ID, t.pose.toPose2d());
      m_aprilTagList.put(t.ID, tagData);
    }
  }

  public static AprilTagFieldLayout getLayout() {
    return aprilTagLayout;
  }

  public static Translation2d getFieldCenter() {
    return fieldCenter;
  }

  public static Pose2d getTag(int id) {
    return m_aprilTagList.get(id).getPose();
  }

  public static void addTarget2d(
      String targetName, Translation2d targetTranslation, int... aprilTagIds) {
    var target = new Target2d(targetName, targetTranslation);
    for (var id : aprilTagIds) {
      target.addAprilTag(id);
      m_aprilTagList.get(id).addRelativeTarget(target);
    }
  }

  public static Optional<Translation2d> getTarget(String name) {
    return getTarget(name, -1);
  }

  public static Optional<Translation2d> getTarget(String name, int nearestTagId) {
    return Optional.ofNullable(m_aprilTagList.get(nearestTagId))
        .map(data -> data.getRelativeTargetTranslation(name))
        .orElse(Optional.ofNullable(m_targetList.get(name).getTranslation()));
  }

  static class Target2d {
    private final String m_name;
    private final Translation2d m_translation;
    private final ArrayList<Integer> m_associatedAprilTags = new ArrayList<>();

    public Target2d(String name, Translation2d translation) {
      m_name = name;
      m_translation = translation;
    }

    public void addAprilTag(int aprilTagId) {
      m_associatedAprilTags.add(aprilTagId);
    }

    public String getName() {
      return m_name;
    }

    public Translation2d getTranslation() {
      return m_translation;
    }

    public ArrayList<Integer> getAssociatedAprilTags() {
      return m_associatedAprilTags;
    }
  }

  static class AprilTagData {
    private final int m_tagId;
    private final Pose2d m_tagPose;
    private final Map<String, Translation2d> m_relativeTargetTranslations = new HashMap<>();

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
      m_relativeTargetTranslations.put(
          target.getName(), target.getTranslation().minus(getPose().getTranslation()));
    }

    public Optional<Translation2d> getRelativeTargetTranslation(String targetName) {
      return Optional.ofNullable(m_relativeTargetTranslations.get(targetName));
    }
  }
}
