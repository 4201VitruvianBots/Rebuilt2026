package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Controls;
import frc.team4201.lib.geometry.FieldRectangle2d;
import frc.team4201.lib.geometry.LinkedAprilTag;
import frc.team4201.lib.geometry.Target3d;
import frc.team4201.lib.simulation.FieldSim;
import frc.team4201.lib.wpilib.AllianceInterface;
import java.util.ArrayList;
import java.util.Collection;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

public class FIELD {
  /**
   * Field constants to use.
   *
   * <p>The default value of the field constants should already be set based on the alliance you are
   * on, which should be set/updated when the robot is disabled.
   */
  private static AprilTagFieldLayout fieldLayout;

  private static Map<String, LinkedAprilTag> aprilTagMap = new HashMap<>();

  private static Distance FIELD_LENGTH;
  private static Distance FIELD_WIDTH;
  private static Translation2d CENTER;
  private static DriverStation.Alliance ALLIANCE;
  private static SECTOR[][] SECTOR_MAP;

  private FIELD() {
    initializeConstants();
  }

  public static void initializeConstants() {
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
    list.add(new LinkedAprilTag("HUB_LEFT", 5, 21, fieldLayout));
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

    ZONE.init();
    HUB.init();
    TOWER.init();
    updateConstants();
  }

  public static void updateConstants() {
    ZONE.updateFields();
    HUB.updateFields();
    TOWER.updateFields();
    ALLIANCE = Controls.getAllianceColor();
    if (ALLIANCE == DriverStation.Alliance.Blue) {
      SECTOR_MAP = SECTOR_MAP_BLUE;
    } else {
      SECTOR_MAP = SECTOR_MAP_RED;
    }
  }

  public static void plotAllPositions(FieldSim fieldSim) {
    fieldSim.addTranslations(
        "Tower targets",
        TOWER.RED.CENTER.getTargetPosition().toTranslation2d(),
        TOWER.RED.LEFT.getTargetPosition().toTranslation2d(),
        TOWER.RED.RIGHT.getTargetPosition().toTranslation2d(),
        TOWER.BLUE.CENTER.getTargetPosition().toTranslation2d(),
        TOWER.BLUE.LEFT.getTargetPosition().toTranslation2d(),
        TOWER.BLUE.RIGHT.getTargetPosition().toTranslation2d());

    fieldSim.addTranslations("Red Hub", HUB.RED.getTargetPosition().toTranslation2d());
    fieldSim.addTranslations("Red Left Bump", ZONE.RED.BUMP.LEFT.getCorners());
    fieldSim.addTranslations("Red Left Trench", ZONE.RED.TRENCH.LEFT.getCorners());
    fieldSim.addTranslations("Red Right Bump", ZONE.RED.BUMP.RIGHT.getCorners());
    fieldSim.addTranslations("Red Right Trench", ZONE.RED.TRENCH.RIGHT.getCorners());
    fieldSim.addTranslations("Blue Hub", HUB.BLUE.getTargetPosition().toTranslation2d());
    fieldSim.addTranslations("Blue Left Bump", ZONE.BLUE.BUMP.LEFT.getCorners());
    fieldSim.addTranslations("Blue Left Trench", ZONE.BLUE.TRENCH.LEFT.getCorners());
    fieldSim.addTranslations("Blue Right Bump", ZONE.BLUE.BUMP.RIGHT.getCorners());
    fieldSim.addTranslations("Blue Right Trench", ZONE.BLUE.TRENCH.RIGHT.getCorners());

    fieldSim.addTranslations(
        "Red Pass Points", TARGET.RED_PASS_POINTS.toArray(new Translation2d[0]));
    fieldSim.addTranslations(
        "Blue Pass Points", TARGET.BLUE_PASS_POINTS.toArray(new Translation2d[0]));
  }

  public enum SECTOR {
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT,
    NEUTRAL_NEAR_LEFT,
    NEUTRAL_NEAR_RIGHT,
    NEUTRAL_FAR_LEFT,
    NEUTRAL_FAR_RIGHT,
    UNKNOWN
  }

  static SECTOR[][] SECTOR_MAP_RED = {
    //    {SECTOR.RED_LEFT, SECTOR.NEUTRAL_NEAR_LEFT, SECTOR.NEUTRAL_FAR_LEFT, SECTOR.BLUE_LEFT},
    {SECTOR.BLUE_LEFT, SECTOR.NEUTRAL_FAR_LEFT, SECTOR.NEUTRAL_NEAR_LEFT, SECTOR.RED_LEFT},
    {SECTOR.BLUE_RIGHT, SECTOR.NEUTRAL_FAR_RIGHT, SECTOR.NEUTRAL_NEAR_RIGHT, SECTOR.RED_RIGHT}
  };
  static SECTOR[][] SECTOR_MAP_BLUE = {
    {SECTOR.BLUE_RIGHT, SECTOR.NEUTRAL_NEAR_RIGHT, SECTOR.NEUTRAL_FAR_RIGHT, SECTOR.RED_RIGHT},
    {SECTOR.BLUE_LEFT, SECTOR.NEUTRAL_NEAR_LEFT, SECTOR.NEUTRAL_FAR_LEFT, SECTOR.RED_LEFT}
  };
  static EnumSet<SECTOR> RED_ALLIANCE_SECTORS = EnumSet.of(SECTOR.RED_LEFT, SECTOR.RED_RIGHT);
  static EnumSet<SECTOR> BLUE_ALLIANCE_SECTORS = EnumSet.of(SECTOR.BLUE_LEFT, SECTOR.BLUE_RIGHT);

  static SECTOR CURRENT_SECTOR = SECTOR.UNKNOWN;

  public static void updateCurrentSector(Pose2d robotPose) {
    /**
     * Get where the robot is on the field. This will be relative to which alliance you are on,
     * which determines which SECTOR_MAP to use, which handles the index -> position mapping based
     * on the order of the sectors in each map.
     */
    int yIdx = robotPose.getY() > CENTER.getY() ? 1 : 0;
    int xIdx = -1;

    if (robotPose.getX() > ZONE.RED_ZONE_LINE.in(Meters)) {
      xIdx = 3;
    } else if (robotPose.getX() > CENTER.getX()) {
      xIdx = 2;
    } else if (robotPose.getX() > ZONE.BLUE_ZONE_LINE.in(Meters)) {
      xIdx = 1;
    } else {
      xIdx = 0;
    }

    // May be redundant/unnecessary to check for unknown
    if (yIdx == -1 || xIdx == -1) {
      CURRENT_SECTOR = SECTOR.UNKNOWN;
    } else {
      CURRENT_SECTOR = SECTOR_MAP[yIdx][xIdx];
    }

    TARGET.updateCurrentTarget(robotPose);
  }

  public static SECTOR getCurrentSector() {
    return CURRENT_SECTOR;
  }

  public static class ZONE implements AllianceInterface {
    public static Class<? extends BASE_ZONE> ALLIANCE;
    public static Class<? extends BASE_ZONE> OPPONENT;

    // Constants our measurements will rely on
    private static final Distance RED_ZONE_LINE =
        aprilTagMap.get("HUB_NEAR").getPose(false).getMeasureX();
    private static final Distance BLUE_ZONE_LINE =
        aprilTagMap.get("HUB_NEAR").getPose(true).getMeasureX();
    private static final Distance HALF_ALLIANCE_ZONE_LENGTH = BLUE_ZONE_LINE.div(2);

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

    public static final FieldRectangle2d NEUTRAL_ZONE =
        new FieldRectangle2d(
            new Translation2d(BLUE_ZONE_LINE, Meters.zero()),
            new Translation2d(RED_ZONE_LINE, FIELD_WIDTH));

    public static void init() {
      RED.init();
      BLUE.init();
    }

    public static class RED extends BASE_ZONE {
      public static FieldRectangle2d ZONE;
      public static FieldRectangle2d DEPOT;

      public static void init() {
        ZONE =
            new FieldRectangle2d(
                new Translation2d(RED_ZONE_LINE, Meters.zero()),
                new Translation2d(FIELD_LENGTH, FIELD_WIDTH));

        DEPOT =
            new FieldRectangle2d(
                new Translation2d(
                    FIELD_LENGTH.minus(DEPOT_DEPTH), RED_DEPOT_CENTER_Y.minus(DEPOT_WIDTH.div(2))),
                new Translation2d(FIELD_LENGTH, RED_DEPOT_CENTER_Y.plus(DEPOT_WIDTH.div(2))));

        BUMP.LEFT =
            new FieldRectangle2d(
                new Translation2d(RED_HUB_X_FAR, TRENCH_WIDTH),
                new Translation2d(RED_HUB_X_NEAR, TRENCH_WIDTH.plus(BUMP_WIDTH)));
        BUMP.RIGHT =
            new FieldRectangle2d(
                new Translation2d(RED_HUB_X_FAR, FIELD_WIDTH.minus(TRENCH_WIDTH.plus(BUMP_WIDTH))),
                new Translation2d(RED_HUB_X_NEAR, FIELD_WIDTH.minus(TRENCH_WIDTH)));

        TRENCH.LEFT =
            new FieldRectangle2d(
                new Translation2d(RED_HUB_X_FAR, Meters.zero()),
                new Translation2d(RED_HUB_X_NEAR, TRENCH_WIDTH));

        TRENCH.RIGHT =
            new FieldRectangle2d(
                new Translation2d(RED_HUB_X_FAR, FIELD_WIDTH.minus(TRENCH_WIDTH)),
                new Translation2d(RED_HUB_X_NEAR, FIELD_WIDTH));
      }
    }

    public static class BLUE extends BASE_ZONE {
      public static FieldRectangle2d ZONE;
      public static FieldRectangle2d DEPOT;

      public static void init() {
        ZONE =
            new FieldRectangle2d(
                Translation2d.kZero, new Translation2d(BLUE_ZONE_LINE, FIELD_WIDTH));

        DEPOT =
            new FieldRectangle2d(
                new Translation2d(Meters.zero(), BLUE_DEPOT_CENTER_Y.minus(DEPOT_WIDTH.div(2))),
                new Translation2d(DEPOT_DEPTH, BLUE_DEPOT_CENTER_Y.plus(DEPOT_WIDTH.div(2))));

        BUMP.LEFT =
            new FieldRectangle2d(
                new Translation2d(
                    BLUE_HUB_X_NEAR, FIELD_WIDTH.minus(TRENCH_WIDTH.plus(BUMP_WIDTH))),
                new Translation2d(BLUE_HUB_X_FAR, FIELD_WIDTH.minus(TRENCH_WIDTH)));

        BUMP.RIGHT =
            new FieldRectangle2d(
                new Translation2d(BLUE_HUB_X_NEAR, TRENCH_WIDTH),
                new Translation2d(BLUE_HUB_X_FAR, TRENCH_WIDTH.plus(BUMP_WIDTH)));

        TRENCH.LEFT =
            new FieldRectangle2d(
                new Translation2d(BLUE_HUB_X_NEAR, FIELD_WIDTH.minus(TRENCH_WIDTH)),
                new Translation2d(BLUE_HUB_X_FAR, FIELD_WIDTH));
        TRENCH.RIGHT =
            new FieldRectangle2d(
                new Translation2d(BLUE_HUB_X_NEAR, Meters.zero()),
                new Translation2d(BLUE_HUB_X_FAR, TRENCH_WIDTH));
      }
    }

    public static void updateFields() {
      if (AllianceInterface.isBlue()) {
        ALLIANCE = BLUE.class;
        OPPONENT = RED.class;
      } else {
        ALLIANCE = RED.class;
        OPPONENT = BLUE.class;
      }
    }

    public static class BASE_ZONE {
      public static BUMP BUMP = new BUMP();
      public static TRENCH TRENCH = new TRENCH();

      public static class BUMP {
        public FieldRectangle2d LEFT;
        public FieldRectangle2d RIGHT;
      }

      public static class TRENCH {
        public FieldRectangle2d LEFT;
        public FieldRectangle2d RIGHT;
      }
    }
  }

  public static class HUB implements AllianceInterface {
    public static final Distance HEIGHT = Inches.of(56.5);
    public static final Distance WIDTH = Inches.of(47.0);

    public static Target3d RED = buildRedGoal();
    public static Target3d BLUE = buildBlueGoal();
    public static Target3d GOAL = RED;

    public static void init() {
      RED = buildRedGoal();
      BLUE = buildBlueGoal();

      updateFields();
    }

    private static Target3d buildRedGoal() {
      return new Target3d(
          new Translation3d(
              aprilTagMap.get("HUB_NEAR").getPose(false).getMeasureX().minus(WIDTH.div(2.0)),
              CENTER.getMeasureY(),
              WIDTH),
          aprilTagMap.entrySet().stream()
              .filter(e -> e.getKey().startsWith("HUB"))
              .mapToInt(e -> e.getValue().getId(false))
              .toArray());
    }

    private static Target3d buildBlueGoal() {
      return new Target3d(
          new Translation3d(
              aprilTagMap.get("HUB_NEAR").getPose(true).getMeasureX().plus(WIDTH.div(2.0)),
              CENTER.getMeasureY(),
              HEIGHT),
          aprilTagMap.entrySet().stream()
              .filter(e -> e.getKey().startsWith("HUB"))
              .mapToInt(e -> e.getValue().getId(true))
              .toArray());
    }

    public static void updateFields() {
      GOAL = Controls.isBlueAlliance() ? BLUE : RED; // TODO: Unflip this T_T
    }
  }

  public static class TOWER implements AllianceInterface {
    public static Target3d CENTER;
    public static Target3d LEFT;
    public static Target3d RIGHT;

    private static final Distance ALLIANCE_WALL_TO_TOWER_FRONT = Inches.of(43.510);
    private static final Distance TOWER_Y_TARGET_OFFSET = Inches.of(22.875);
    private static final Distance TARGET_X_OFFSET =
        ALLIANCE_WALL_TO_TOWER_FRONT.plus(ROBOT.ROBOTLENGTH.div(2.0)).plus(ROBOT.BUMPERTHICKNESS);

    public static void init() {
      RED.init();
      BLUE.init();
    }

    public static class RED {
      public static Pose3d APRILTAG;
      public static Target3d CENTER;
      public static Target3d LEFT;
      public static Target3d RIGHT;

      public static void init() {
        APRILTAG = aprilTagMap.get("TOWER").getPose(false);

        CENTER =
            new Target3d(
                new Translation3d(
                    APRILTAG.getMeasureX().minus(TARGET_X_OFFSET),
                    APRILTAG.getMeasureY(),
                    Meters.zero()),
                aprilTagMap.entrySet().stream()
                    .filter(e -> e.getKey().startsWith("TOWER"))
                    .mapToInt(e -> e.getValue().getId(false))
                    .toArray());

        LEFT =
            new Target3d(
                new Translation3d(
                    APRILTAG.getMeasureX().minus(TARGET_X_OFFSET),
                    APRILTAG.getMeasureY().minus(TOWER_Y_TARGET_OFFSET),
                    Meters.zero()),
                aprilTagMap.entrySet().stream()
                    .filter(e -> e.getKey().startsWith("TOWER"))
                    .mapToInt(e -> e.getValue().getId(false))
                    .toArray());

        RIGHT =
            new Target3d(
                new Translation3d(
                    APRILTAG.getMeasureX().minus(TARGET_X_OFFSET),
                    APRILTAG.getMeasureY().plus(TOWER_Y_TARGET_OFFSET),
                    Meters.zero()),
                aprilTagMap.entrySet().stream()
                    .filter(e -> e.getKey().startsWith("TOWER"))
                    .mapToInt(e -> e.getValue().getId(false))
                    .toArray());
      }
    }

    public static class BLUE {
      public static Pose3d APRILTAG;
      public static Target3d CENTER;
      public static Target3d LEFT;
      public static Target3d RIGHT;

      public static void init() {
        APRILTAG = aprilTagMap.get("TOWER").getPose(true);

        CENTER =
            new Target3d(
                new Translation3d(
                    APRILTAG.getMeasureX().plus(TARGET_X_OFFSET),
                    APRILTAG.getMeasureY(),
                    Meters.zero()),
                aprilTagMap.entrySet().stream()
                    .filter(e -> e.getKey().startsWith("TOWER"))
                    .mapToInt(e -> e.getValue().getId(true))
                    .toArray());
        LEFT =
            new Target3d(
                new Translation3d(
                    APRILTAG.getMeasureX().plus(TARGET_X_OFFSET),
                    APRILTAG.getMeasureY().plus(TOWER_Y_TARGET_OFFSET),
                    Meters.zero()),
                aprilTagMap.entrySet().stream()
                    .filter(e -> e.getKey().startsWith("TOWER"))
                    .mapToInt(e -> e.getValue().getId(true))
                    .toArray());

        RIGHT =
            new Target3d(
                new Translation3d(
                    APRILTAG.getMeasureX().plus(TARGET_X_OFFSET),
                    APRILTAG.getMeasureY().minus(TOWER_Y_TARGET_OFFSET),
                    Meters.zero()),
                aprilTagMap.entrySet().stream()
                    .filter(e -> e.getKey().startsWith("TOWER"))
                    .mapToInt(e -> e.getValue().getId(true))
                    .toArray());
      }
    }

    public static void updateFields() {
      if (AllianceInterface.isBlue()) {
        CENTER = BLUE.CENTER;
        LEFT = BLUE.LEFT;
        RIGHT = BLUE.RIGHT;
      } else {
        CENTER = RED.CENTER;
        LEFT = RED.LEFT;
        RIGHT = RED.RIGHT;
      }
    }
  }

  public static class TARGET implements AllianceInterface {
    public static Target3d CURRENT_TARGET;
    public static Target3d RED_LEFT_PASS =
        new Target3d(
            new Translation3d(
                FIELD_LENGTH.minus(ZONE.HALF_ALLIANCE_ZONE_LENGTH.times(0.85)),
                FIELD_WIDTH.times(0.2),
                Meters.zero()));

    public static Target3d RED_RIGHT_PASS =
        new Target3d(
            new Translation3d(
                FIELD_LENGTH.minus(ZONE.HALF_ALLIANCE_ZONE_LENGTH.times(0.85)),
                FIELD_WIDTH.times(0.8),
                Meters.zero()));

    public static Collection<Translation2d> RED_PASS_POINTS =
        List.of(
            RED_LEFT_PASS.getTargetPosition().toTranslation2d(),
            RED_RIGHT_PASS.getTargetPosition().toTranslation2d());

    public static Target3d BLUE_LEFT_PASS =
        new Target3d(
            new Translation3d(
                ZONE.HALF_ALLIANCE_ZONE_LENGTH.times(0.85), FIELD_WIDTH.times(0.8), Meters.zero()));

    public static Target3d BLUE_RIGHT_PASS =
        new Target3d(
            new Translation3d(
                ZONE.HALF_ALLIANCE_ZONE_LENGTH.times(0.85), FIELD_WIDTH.times(0.2), Meters.zero()));

    public static Collection<Translation2d> BLUE_PASS_POINTS =
        List.of(
            BLUE_LEFT_PASS.getTargetPosition().toTranslation2d(),
            BLUE_RIGHT_PASS.getTargetPosition().toTranslation2d());

    public static Map<Translation2d, Target3d> TRANSLATION_TO_TARGET =
        Map.ofEntries(
            Map.entry(RED_LEFT_PASS.getTargetPosition().toTranslation2d(), RED_LEFT_PASS),
            Map.entry(RED_RIGHT_PASS.getTargetPosition().toTranslation2d(), RED_RIGHT_PASS),
            Map.entry(BLUE_LEFT_PASS.getTargetPosition().toTranslation2d(), BLUE_LEFT_PASS),
            Map.entry(BLUE_RIGHT_PASS.getTargetPosition().toTranslation2d(), BLUE_RIGHT_PASS));

    public static void updateCurrentTarget(Pose2d robotPose) {
      if (AllianceInterface.isBlue()) {
        if (BLUE_ALLIANCE_SECTORS.contains(getCurrentSector())) {
          CURRENT_TARGET = HUB.BLUE;
        } else {
          CURRENT_TARGET =
              TRANSLATION_TO_TARGET.get(robotPose.getTranslation().nearest(BLUE_PASS_POINTS));
        }
      } else {
        if (RED_ALLIANCE_SECTORS.contains(getCurrentSector())) {
          CURRENT_TARGET = HUB.RED;
        } else {
          CURRENT_TARGET =
              TRANSLATION_TO_TARGET.get(robotPose.getTranslation().nearest(RED_PASS_POINTS));
        }
      }
    }
  }

  public enum BUMP_ALIGNMENT_TARGETS {
    // Naming scheme is as follows:
    // First word is side
    // Second word is whether the pose is in the neutral or alliance zone
    // Third word is what these poses are in relation to
    // All units are in meters
    // Everything has been measured from our auto waypoints based on the welded field
    LEFT_ALLIANCE_BUMP(new Pose2d(3.586, 5.536, Rotation2d.kZero)),
    RIGHT_ALLIANCE_BUMP(new Pose2d(3.586, 2.533, Rotation2d.kZero)),
    LEFT_ALLIANCE_SHOOTING(new Pose2d(2.974, 5.536, new Rotation2d(-42.557))),
    RIGHT_ALLIANCE_SHOOTING(new Pose2d(2.974, 2.533, new Rotation2d(42.557))),
    LEFT_NEUTRAL_BUMP(new Pose2d(5.833, 5.536, Rotation2d.kZero)),
    RIGHT_NEUTRAL_BUMP(new Pose2d(5.833, 2.533, Rotation2d.kZero)),
    RIGHT_UNREALISTIC_POSE(new Pose2d(0.0, 2.354, Rotation2d.kZero)),
    LEFT_UNREALISTIC_POSE(new Pose2d(0.0, 5.536, Rotation2d.kZero));

    private final Pose2d pose2d;

    BUMP_ALIGNMENT_TARGETS(final Pose2d pose2d) {
      this.pose2d = pose2d;
    }

    public Pose2d getAlignmentPose() {
      return pose2d;
    }
  }
}
