package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.Arrays;

public class FIELD {
  /**
   * FIELD
   *
   * <p>Field constants
   *
   * <p>Note: Values are using ideal values from WPILib TODO: Create layout from practice field.
   */
  public static final AprilTagFieldLayout wpilibAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // public static final AprilTagFieldLayout practiceFieldAprilTagLayout;

  public static final AprilTagFieldLayout aprilTagFieldLayout = wpilibAprilTagLayout;

  public static final Translation2d blueHub = new Translation2d(4.625594, 4.034536);
  public static final Translation2d redHub = new Translation2d(11.915394, 4.034536);

  public static final Translation2d redAutoHub = redHub;
  public static final Translation2d blueAutoHub = blueHub;

  /** Field X-axis */
  public static final Distance LENGTH = Meters.of(aprilTagFieldLayout.getFieldLength());

  /** Field Y-axis */
  public static final Distance WIDTH = Meters.of(aprilTagFieldLayout.getFieldWidth());

  public static final Distance APRILTAG_SIZE = Inches.of(6.5);

  public static final Distance HUB_APRILTAG_HEIGHT = Inches.of(44.25);
  public static final Distance TRENCH_APRILTAG_HEIGHT = Inches.of(35);
  public static final Distance TOWER_APRILTAG_HEIGHT = Inches.of(33).plus(APRILTAG_SIZE).div(2.0);
  public static final Distance OUTPOST_APRILTAG_HEIGHT = Inches.of(33).plus(APRILTAG_SIZE).div(2.0);

  /** Enum describing all AprilTags on the field by ID and their Pose3d positions. */
  public enum APRIL_TAG {
    RED_TRENCH_NEAR_RIGHT(1),
    RED_HUB_NEAR_RIGHT(2),
    RED_HUB_RIGHT_CLOSE(3),
    RED_HUB_RIGHT_FAR(4),
    RED_HUB_FAR_RIGHT(5),
    RED_TRENCH_FAR_RIGHT(6),
    RED_TRENCH_FAR_LEFT(7),
    RED_HUB_FAR_LEFT(8),
    RED_HUB_LEFT_AWAY(9),
    RED_HUB_LEFT_CLOSE(10),
    RED_HUB_NEAR_LEFT(11),
    RED_TRENCH_NEAR_LEFT(12),
    RED_OUTPOST_NEAR(13),
    RED_OUTPOST_FAR(14),
    RED_TOWER_NEAR(15),
    RED_TOWER_FAR(16),
    BLUE_TRENCH_FAR_LEFT(17),
    BLUE_HUB_FAR_LEFT(18),
    BLUE_HUB_LEFT_FAR(19),
    BLUE_HUB_LEFT_NEAR(20),
    BLUE_HUB_NEAR_LEFT(21),
    BLUE_TRENCH_NEAR_LEFT(22),
    BLUE_TRENCH_NEAR_RIGHT(23),
    BLUE_HUB_NEAR_RIGHT(24),
    BLUE_HUB_RIGHT_NEAR(25),
    BLUE_HUB_RIGHT_FAR(26),
    BLUE_HUB_FAR_RIGHT(27),
    BLUE_TRENCH_FAR_RIGHT(28),
    BLUE_OUTPOST_FAR(29),
    BLUE_OUTPOST_NEAR(30),
    BLUE_TOWER_FAR(31),
    BLUE_TOWER_NEAR(32);

    private final int id;
    private Pose3d pose;

    APRIL_TAG(final int id) {
      this.id = id;
      aprilTagFieldLayout
          .getTagPose(this.id)
          .ifPresentOrElse(
              pose3d -> this.pose = pose3d,
              () -> {
                System.out.printf(
                    "[FIELD] Could not read AprilTag ID %s data from FieldLayout\n", this.id);
                new Alert(
                        String.format("[FIELD] APRIL_TAG ID %s value couldn't be read", this.id),
                        AlertType.kError)
                    .set(true);
              });

      if (this.pose == null) {
        throw new IllegalArgumentException("AprilTag ID " + this.id + " does not have a Pose3d!");
      }
    }

    public int getId() {
      return id;
    }

    public Pose3d getPose3d() {
      return pose;
    }

    public Pose2d getPose2d() {
      return pose.toPose2d();
    }

    public Translation3d getTranslation3d() {
      return pose.getTranslation();
    }

    public Translation2d getTranslation2d() {
      return getPose2d().getTranslation();
    }

    public static APRIL_TAG getTagById(int aprilTagId) {
      for (var t : APRIL_TAG.values()) {
        if (t.id == aprilTagId) {
          return t;
        }
      }
      throw new IllegalArgumentException("AprilTag ID " + aprilTagId + " does not exist!");
    }

    public static Pose2d[] getAllAprilTagPoses() {
      return Arrays.stream(APRIL_TAG.values()).map(APRIL_TAG::getPose2d).toArray(Pose2d[]::new);
    }
  }
}
