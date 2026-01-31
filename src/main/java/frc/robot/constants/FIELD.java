package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4201.lib.geometry.LinkedAprilTag;
import frc.team4201.lib.geometry.Target3d;
import frc.team4201.lib.wpilib.AllianceInterface;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

public class FIELD {
  private static AprilTagFieldLayout fieldLayout;
  private static Map<String, LinkedAprilTag> aprilTagMap = new HashMap<>();

  private static Distance FIELD_LENGTH;
  private static Distance FIELD_WIDTH;
  private static Translation2d CENTER;

  private FIELD() {
    if (DriverStation.isFMSAttached()) {
      fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    } else {
      fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    }

    FIELD_LENGTH = Meters.of(fieldLayout.getFieldLength());
    FIELD_WIDTH = Meters.of(fieldLayout.getFieldWidth());
    CENTER = new Translation2d(FIELD_LENGTH.div(2.0), FIELD_WIDTH.div(2.0));

    // Build AprilTag Map
    List<LinkedAprilTag> list = new ArrayList<>();
    list.add(new LinkedAprilTag("RIGHT_TRENCH_FAR", 1, 17, fieldLayout));
    list.add(new LinkedAprilTag("HUB_RIGHT", 2, 18, fieldLayout));
    list.add(new LinkedAprilTag("HUB_FAR_SECONDARY", 3, 19, fieldLayout));
    list.add(new LinkedAprilTag("HUB_FAR", 4, 20, fieldLayout));
    list.add(new LinkedAprilTag("HUB_RIGHT", 5, 21, fieldLayout));
    list.add(new LinkedAprilTag("LEFT_TRENCH_FAR", 6, 22, fieldLayout));
    list.add(new LinkedAprilTag("LEFT_TRENCH_NEAR", 7, 23, fieldLayout));
    list.add(new LinkedAprilTag("HUB_RIGHT_SECONDARY", 8, 24, fieldLayout));
    list.add(new LinkedAprilTag("HUB_NEAR_SECONDARY", 9, 25, fieldLayout));
    list.add(new LinkedAprilTag("HUB_NEAR", 10, 26, fieldLayout));
    list.add(new LinkedAprilTag("HUB_RIGHT_NEAR", 11, 27, fieldLayout));
    list.add(new LinkedAprilTag("TRENCH_RIGHT_NEAR", 12, 28, fieldLayout));
    list.add(new LinkedAprilTag("OUTPOST", 13, 29, fieldLayout));
    list.add(new LinkedAprilTag("OUTPOST_SECONDARY", 14, 30, fieldLayout));
    list.add(new LinkedAprilTag("TOWER", 15, 31, fieldLayout));
    list.add(new LinkedAprilTag("TOWER_SECONDARY", 16, 32, fieldLayout));
    aprilTagMap =
        list.stream().collect(Collectors.toMap(LinkedAprilTag::getName, Function.identity()));

    Target3d.loadField(fieldLayout);
  }

  public void plotAllPositions() {}

  static class ZONE implements AllianceInterface {
    public static Rectangle2d ALLIANCE_ZONE;
    public static Rectangle2d OPPONENT_ZONE;
    public static Rectangle2d ALLIANCE_DEPOT;
    public static Rectangle2d OPPONENT_DEPOT;
    public static Rectangle2d ALLIANCE_LEFT_BUMP;
    public static Rectangle2d ALLIANCE_LEFT_TRENCH;
    public static Rectangle2d ALLIANCE_RIGHT_BUMP;
    public static Rectangle2d ALLIANCE_RIGHT_TRENCH;
    public static Rectangle2d OPPONENT_LEFT_BUMP;
    public static Rectangle2d OPPONENT_LEFT_TRENCH;
    public static Rectangle2d OPPONENT_RIGHT_BUMP;
    public static Rectangle2d OPPONENT_RIGHT_TRENCH;

    // Constants our measurements will rely on
    private static final Distance RED_ZONE_LINE =
        aprilTagMap.get("HUB_NEAR").getPose(false).getMeasureX();
    private static final Distance BLUE_ZONE_LINE =
        aprilTagMap.get("HUB_NEAR").getPose(true).getMeasureX();

    private static final Distance DEPOT_WIDTH = Inches.of(42);
    private static final Distance DEPOT_DEPTH = Inches.of(27);

    private static final Distance TRENCH_WIDTH = Inches.of(65.65);
    private static final Distance TRENCH_DEPTH = Inches.of(47.0);

    private static final Distance BUMP_WIDTH = Inches.of(73.0);
    private static final Distance BUMP_DEPTH = Inches.of(44.4);

    private static final Distance RED_HUB_X_NEAR = RED_ZONE_LINE;
    private static final Distance RED_HUB_X_FAR =
        aprilTagMap.get("HUB_FAR").getPose(false).getMeasureX();
    private static final Distance RED_DEPOT_CENTER_Y = CENTER.getMeasureY().plus(Inches.of(75.93));

    private static final Distance BLUE_HUB_X_NEAR = BLUE_ZONE_LINE;
    private static final Distance BLUE_HUB_X_FAR =
        aprilTagMap.get("HUB_FAR").getPose(true).getMeasureX();
    private static final Distance BLUE_DEPOT_CENTER_Y =
        CENTER.getMeasureY().minus(Inches.of(75.93));

    public static final Rectangle2d NEUTRAL_ZONE =
            new Rectangle2d(
                    new Translation2d(BLUE_ZONE_LINE, Meters.zero()),
                    new Translation2d(RED_ZONE_LINE, FIELD_WIDTH));

    // Red Alliance Zones
    private static final Rectangle2d RED_ALLIANCE_ZONE =
        new Rectangle2d(
            new Translation2d(RED_ZONE_LINE, Meters.zero()),
            new Translation2d(FIELD_LENGTH, FIELD_WIDTH));

    private static final Rectangle2d RED_DEPOT =
        new Rectangle2d(
            new Translation2d(
                FIELD_LENGTH.minus(DEPOT_DEPTH), RED_DEPOT_CENTER_Y.minus(DEPOT_WIDTH.div(2))),
            new Translation2d(FIELD_LENGTH, RED_DEPOT_CENTER_Y.plus(DEPOT_WIDTH.div(2))));

    private static final Rectangle2d RED_LEFT_BUMP =
        new Rectangle2d(
            new Translation2d(RED_HUB_X_FAR, TRENCH_WIDTH),
            new Translation2d(RED_HUB_X_NEAR, TRENCH_WIDTH.plus(BUMP_WIDTH)));
    private static final Rectangle2d RED_RIGHT_BUMP =
        new Rectangle2d(
            new Translation2d(RED_HUB_X_FAR, FIELD_WIDTH.minus(TRENCH_WIDTH.plus(BUMP_WIDTH))),
            new Translation2d(RED_HUB_X_NEAR, FIELD_WIDTH.minus(TRENCH_WIDTH)));
    private static final Rectangle2d RED_LEFT_TRENCH =
        new Rectangle2d(
            new Translation2d(RED_HUB_X_FAR, Meters.zero()),
            new Translation2d(RED_HUB_X_NEAR, TRENCH_WIDTH));
    private static final Rectangle2d RED_RIGHT_TRENCH =
        new Rectangle2d(
            new Translation2d(RED_HUB_X_FAR, FIELD_WIDTH.minus(TRENCH_WIDTH)),
            new Translation2d(RED_HUB_X_NEAR, FIELD_WIDTH));

    private static final Rectangle2d BLUE_ALLIANCE_ZONE =
        new Rectangle2d(Translation2d.kZero, new Translation2d(BLUE_ZONE_LINE, FIELD_WIDTH));

    private static final Rectangle2d BLUE_DEPOT =
        new Rectangle2d(
            new Translation2d(Meters.zero(), BLUE_DEPOT_CENTER_Y.minus(DEPOT_WIDTH.div(2))),
            new Translation2d(DEPOT_DEPTH, BLUE_DEPOT_CENTER_Y.plus(DEPOT_WIDTH.div(2))));
    private static final Rectangle2d BLUE_LEFT_BUMP =
        new Rectangle2d(
            new Translation2d(BLUE_HUB_X_NEAR, FIELD_WIDTH.minus(TRENCH_WIDTH.plus(BUMP_WIDTH))),
            new Translation2d(BLUE_HUB_X_FAR, FIELD_WIDTH.minus(TRENCH_WIDTH)));
    private static final Rectangle2d BLUE_RIGHT_BUMP =
        new Rectangle2d(
            new Translation2d(BLUE_HUB_X_NEAR, TRENCH_WIDTH),
            new Translation2d(BLUE_HUB_X_FAR, TRENCH_WIDTH.plus(BUMP_WIDTH)));
    private static final Rectangle2d BLUE_LEFT_TRENCH =
        new Rectangle2d(
            new Translation2d(BLUE_HUB_X_NEAR, FIELD_WIDTH.minus(TRENCH_WIDTH)),
            new Translation2d(BLUE_HUB_X_FAR, FIELD_WIDTH));
    private static final Rectangle2d BLUE_RIGHT_TRENCH =
        new Rectangle2d(
            new Translation2d(BLUE_HUB_X_NEAR, Meters.zero()),
            new Translation2d(BLUE_HUB_X_FAR, TRENCH_WIDTH));

    @Override
    public void updateFields() {
      if (AllianceInterface.isBlue()) {
        ALLIANCE_ZONE = BLUE_ALLIANCE_ZONE;
        ALLIANCE_DEPOT = BLUE_DEPOT;
        OPPONENT_ZONE = RED_ALLIANCE_ZONE;
        OPPONENT_DEPOT = RED_DEPOT;
        ALLIANCE_LEFT_BUMP = BLUE_LEFT_BUMP;
        ALLIANCE_LEFT_TRENCH = BLUE_LEFT_TRENCH;
        ALLIANCE_RIGHT_BUMP = BLUE_RIGHT_BUMP;
        ALLIANCE_RIGHT_TRENCH = BLUE_RIGHT_TRENCH;
        OPPONENT_LEFT_BUMP = RED_LEFT_BUMP;
        OPPONENT_LEFT_TRENCH = RED_LEFT_TRENCH;
        OPPONENT_RIGHT_BUMP = RED_RIGHT_BUMP;
        OPPONENT_RIGHT_TRENCH = RED_RIGHT_TRENCH;
      } else {
        ALLIANCE_ZONE = RED_ALLIANCE_ZONE;
        ALLIANCE_DEPOT = RED_DEPOT;
        OPPONENT_ZONE = BLUE_ALLIANCE_ZONE;
        OPPONENT_DEPOT = BLUE_DEPOT;
        ALLIANCE_LEFT_BUMP = RED_LEFT_BUMP;
        ALLIANCE_LEFT_TRENCH = RED_LEFT_TRENCH;
        ALLIANCE_RIGHT_BUMP = RED_RIGHT_BUMP;
        ALLIANCE_RIGHT_TRENCH = RED_RIGHT_TRENCH;
        OPPONENT_LEFT_BUMP = BLUE_LEFT_BUMP;
        OPPONENT_LEFT_TRENCH = BLUE_LEFT_TRENCH;
        OPPONENT_RIGHT_BUMP = BLUE_RIGHT_BUMP;
        OPPONENT_RIGHT_TRENCH = BLUE_RIGHT_TRENCH;
      }
    }
  }

  static class HUB implements AllianceInterface {
    public static final Distance HEIGHT = Inches.of(56.5);
    public static final Distance WIDTH = Inches.of(47.0);

    public static Target3d GOAL;

    private static final Target3d RED =
        new Target3d(
            new Translation3d(
                aprilTagMap.get("HUB_NEAR").getPose(false).getMeasureX().plus(WIDTH.div(2.0)),
                CENTER.getMeasureY(),
                WIDTH),
            aprilTagMap.entrySet().stream()
                .filter(e -> e.getKey().startsWith("HUB"))
                .mapToInt(e -> e.getValue().getId(false))
                .toArray());

    private static final Target3d BLUE =
        new Target3d(
            new Translation3d(
                aprilTagMap.get("HUB_NEAR").getPose(true).getMeasureX().plus(WIDTH.div(2.0)),
                CENTER.getMeasureY(),
                HEIGHT),
            aprilTagMap.entrySet().stream()
                .filter(e -> e.getKey().startsWith("HUB"))
                .mapToInt(e -> e.getValue().getId(true))
                .toArray());

    @Override
    public void updateFields() {
      GOAL = AllianceInterface.isBlue() ? BLUE : RED;
    }
  }
}
