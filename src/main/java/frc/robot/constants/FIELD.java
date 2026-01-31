package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4201.lib.geometry.Target3d;
import frc.team4201.lib.geometry.Targeting2d;
import frc.team4201.lib.simulation.FieldSim;

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
  private static Target3d redTowerCenterGoal;
  private static Target3d redTowerLeftGoal;
  private static Target3d redTowerRightGoal;
  private static Target3d blueTowerCenterGoal;
  private static Target3d blueTowerLeftGoal;
  private static Target3d blueTowerRightGoal;

  public static void buildFieldConstants() {
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
                field.getTagPose(10).get().getX() - hubWidth.in(Meters) / 2.0,
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
                field.getTagPose(26).get().getX() + hubWidth.in(Meters) / 2.0,
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

    Distance allianceWallToTowerFront = Inches.of(43.510);
    Distance allianceWallToTowerSide = Inches.of(22.875);

    redTowerCenterGoal =
        new Target3d(
            new Translation3d(
                Meters.of(field.getTagPose(15).get().getX())
                .minus(ROBOT.ROBOTLENGTH
                    .div(2.0))
                    .minus(ROBOT.BUMPERTHICKNESS)
                    .minus(allianceWallToTowerFront)
                    .in(Meters),
                field.getTagPose(15).get().getY(),
                0),
            15,
            16);

    redTowerLeftGoal =
        new Target3d(
            new Translation3d(
                Meters.of(field.getTagPose(15).get().getX())
                .minus(ROBOT.ROBOTLENGTH
                    .div(2.0))
                    .minus(ROBOT.BUMPERTHICKNESS)
                    .minus(allianceWallToTowerFront)
                    .in(Meters),
                Meters.of(field.getTagPose(15).get().getY())
                .minus(allianceWallToTowerSide)
                .in(Meters),
                0),
            15,
            16);
    
    redTowerRightGoal =
        new Target3d(
            new Translation3d(
                Meters.of(field.getTagPose(15).get().getX())
                .minus(ROBOT.ROBOTLENGTH
                    .div(2.0))
                    .minus(ROBOT.BUMPERTHICKNESS)
                    .minus(allianceWallToTowerFront)
                    .in(Meters),
                Meters.of(field.getTagPose(15).get().getY())
                .plus(allianceWallToTowerSide)
                .in(Meters),
                0),
            15,
            16);

    blueTowerCenterGoal =
        new Target3d(
            new Translation3d(
                ROBOT
                    .ROBOTLENGTH
                    .div(2.0)
                    .plus(ROBOT.BUMPERTHICKNESS)
                    .plus(allianceWallToTowerFront)
                    .plus(Meters.of(field.getTagPose(31).get().getX()))
                    .in(Meters),
                field.getTagPose(31).get().getY(),
                0),
            31,
            32);

    blueTowerLeftGoal =
        new Target3d(
            new Translation3d(
                ROBOT
                    .ROBOTLENGTH
                    .div(2.0)
                    .plus(ROBOT.BUMPERTHICKNESS)
                    .plus(allianceWallToTowerFront)
                    .plus(Meters.of(field.getTagPose(31).get().getX()))
                    .in(Meters),
                Meters.of(field.getTagPose(31).get().getY())
                .plus(allianceWallToTowerSide)
                .in(Meters),
                0),
            31,
            32);
    
    blueTowerRightGoal =
        new Target3d(
            new Translation3d(
                ROBOT
                    .ROBOTLENGTH
                    .div(2.0)
                    .plus(ROBOT.BUMPERTHICKNESS)
                    .plus(allianceWallToTowerFront)
                    .plus(Meters.of(field.getTagPose(31).get().getX()))
                    .in(Meters),
                Meters.of(field.getTagPose(31).get().getY())
                .minus(allianceWallToTowerSide)
                .in(Meters),
                0),
            31,
            32);
  }

  public static void plotAllPositions(FieldSim fieldsim) {
    fieldsim.addPoses("Tower targets", 
    new Pose2d(redTowerCenterGoal.getTargetPosition().toTranslation2d(), Rotation2d.kZero),
    new Pose2d(redTowerLeftGoal.getTargetPosition().toTranslation2d(), Rotation2d.kZero),
    new Pose2d(redTowerRightGoal.getTargetPosition().toTranslation2d(), Rotation2d.kZero),
    new Pose2d(blueTowerCenterGoal.getTargetPosition().toTranslation2d(), Rotation2d.kZero),
    new Pose2d(blueTowerLeftGoal.getTargetPosition().toTranslation2d(), Rotation2d.kZero),
    new Pose2d(blueTowerRightGoal.getTargetPosition().toTranslation2d(), Rotation2d.kZero));
  }

  // public enum TOWER{
  //   RED(),
  //   BLUE();

  //   TOWER(final int aprilTagId, Distance offset) {
  //     Pose2d aprilTagPose = Pose2d.kZero;

  //     try {
  //       aprilTagPose = APRIL_TAG.getTagById(aprilTagId).getPose2d();
  //     } catch (Exception e) {
  //       System.out.printf(
  //           "[FIELD] Could not get AprilTag %d Pose for ALGAE_TARGET generation!\n", aprilTagId);
  //     }

  //     // TODO: Implement
  //     //      Translation2d targetOffset = Translation2d.kZero;
  //     //      switch (type) {
  //     //        case REEF -> targetOffset = baseReefAlgaeTargetOffset;
  //     //        case PROCESSOR -> targetOffset = Translation2d.kZero;
  //     //        case BARGE -> targetOffset = Translation2d.kZero;
  //     //        case STAGED -> targetOffset = Translation2d.kZero;
  //     //      }

  //     Pose2d targetPose =
  //         aprilTagPose.plus(
  //             new Transform2d(
  //                 baseAlgaeTargetOffset.plus(new Translation2d(offset.in(Meters), 0)),
  //                 Rotation2d.k180deg));

  //     this.pose = aprilTagPose;
  //     this.targetPose = targetPose;
  //   }
  // }
}
