package frc.team4201.lib.geometry;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.HashMap;
import java.util.Map;

public class Target3d {
  private static AprilTagFieldLayout m_field;

  private final Translation3d m_targetPosition;
  private final Map<Integer, Translation3d> m_tagToTarget = new HashMap<>();

  public static void loadField(AprilTagFieldLayout field) {
    m_field = field;
  }

  public Target3d(Translation3d targetPosition, int... aprilTagIds) {
    m_targetPosition = targetPosition;

    for (int i : aprilTagIds) {
      withTag(i);
    }
  }

  public Target3d(Translation3d targetPosition) {
    m_targetPosition = targetPosition;
  }

  public Target3d withTag(int id) {
    m_field
        .getTagPose(id)
        .ifPresentOrElse(
            t -> m_tagToTarget.put(id, m_targetPosition.minus(t.getTranslation())),
            () -> {
              throw new IllegalArgumentException("Invalid AprilTag ID given!");
            });

    return this;
  }

  public Translation3d getTargetPosition() {
    return getTargetPosition(-1, Translation3d.kZero);
  }

  public Translation3d getTargetPosition(int id, Translation3d tagPosition) {
    if (m_tagToTarget.containsKey(id)) {
      return tagPosition.plus(m_tagToTarget.get(id));
    } else {
      return m_targetPosition;
    }
  }
}
