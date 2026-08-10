package frc.team4201.lib.geometry;

import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;

public class LinkedAprilTag {
  private final String tagName;
  private final int redId;
  private final int blueId;
  private Pose3d bluePose;
  private Pose3d redPose;

  public LinkedAprilTag(String name, int redId, int blueId, AprilTagFieldLayout field) {
    tagName = name;
    this.redId = redId;
    this.blueId = blueId;
    field.getTagPose(redId).ifPresent(p -> redPose = p);
    field.getTagPose(blueId).ifPresent(p -> bluePose = p);
  }

  public String getName() {
    return tagName;
  }

  public int getId() {
    return getId(isBlue());
  }

  public int getId(Boolean useBlue) {
    return useBlue ? blueId : redId;
  }

  public Pose3d getPose() {
    return getPose(isBlue());
  }

  public Pose3d getPose(Boolean useBlue) {
    return useBlue ? bluePose : redPose;
  }

  private boolean isBlue() {
    if (DriverStationBackend.getAlliance().isPresent()) {
      return DriverStationBackend.getAlliance().get().equals(Alliance.BLUE);
    } else {
      return false;
    }
  }
}
