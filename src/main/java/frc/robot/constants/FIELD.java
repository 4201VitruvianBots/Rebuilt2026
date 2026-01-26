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
import frc.team4201.lib.geometry.Target3d;
import frc.team4201.lib.geometry.Targeting2d;

public class FIELD {
  private static AprilTagFieldLayout field;

  private static Translation2d fieldCenter;
  private static Distance redZoneLine;
  private static Rectangle2d redZoneArea;
  private static Distance blueZoneLine;
  private static Rectangle2d blueZoneArea;
  private static Rectangle2d neutralZoneArea;

  private static final Distance hubHeight = Inches.of(56.5);
  private static final Distance hubWidth = Inches.of(47.0);

  private static Target3d redHubGoal;
  private static Target3d blueHubGoal;

  public void buildFieldConstants() {
    if (DriverStation.isFMSAttached()) {
      field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    } else {
      field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    }
    fieldCenter = new Translation2d(field.getFieldLength() / 2.0, field.getFieldWidth() / 2.0);
    field.getTagPose(10).ifPresent(t -> redZoneLine = Meters.of(t.getX()));
    field.getTagPose(26).ifPresent(t -> blueZoneLine = Meters.of(t.getX()));

    redZoneArea =
        new Rectangle2d(
            new Translation2d(redZoneLine.in(Meters), 0),
            new Translation2d(field.getFieldLength(), field.getFieldWidth()));
    blueZoneArea =
        new Rectangle2d(
            new Translation2d(0, 0),
            new Translation2d(blueZoneLine.in(Meters), field.getFieldWidth()));
    neutralZoneArea =
        new Rectangle2d(
            new Translation2d(blueZoneLine.in(Meters), 0),
            new Translation2d(redZoneLine.in(Meters), field.getFieldWidth()));

    // Targeting Values
    Target3d.loadField(field);

    // Initialize a base position for the hub goals
    redHubGoal =
        new Target3d(
            new Translation3d(
                Targeting2d.getTag(10).getX() - hubWidth.in(Meters) / 2.0,
                fieldCenter.getY(),
                hubHeight.in(Meters)),
            2,
            3,
            4,
            5,
            8,
            9,
            10,
            11);

    blueHubGoal =
        new Target3d(
            new Translation3d(
                Targeting2d.getTag(26).getX() + hubWidth.in(Meters) / 2.0,
                fieldCenter.getY(),
                hubHeight.in(Meters)),
            18,
            19,
            20,
            20,
            21,
            24,
            25,
            26,
            27);
  }

  public void plotAllPositions() {

  }
}
