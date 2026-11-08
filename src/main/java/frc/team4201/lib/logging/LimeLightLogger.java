package frc.team4201.lib.logging;

import com.ctre.phoenix6.Utils;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimate;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

public class LimeLightLogger {
  private final TelemetryTable telemetry;

  public LimeLightLogger(Limelight limelight) {
    this.telemetry = Telemetry.getTable("llTable").getTable(limelight.getName());
  }

  public void logData(PoseEstimate measurement, boolean isResultValid) {
    publishValid(isResultValid);
    if (measurement.isValid() && isResultValid) {
      publishTimestamp(measurement.timestampSeconds);
      publishRobotTimestamp(Utils.getCurrentTimeSeconds());
      publishPose(measurement.pose);
      publishTagCount(measurement.reportedTagCount);
      publishMegatag2Pose(measurement.isMT2());
    } else {
      publishTimestamp(-1);
      publishRobotTimestamp(-1);
      publishPose(new Pose2d(-1, -1, Rotation2d.ZERO));
      publishTagCount(-1);
      publishMegatag2Pose(false);
    }
  }

  private void publishTimestamp(double timestamp) {
    telemetry.log("estTimestamp", timestamp);
  }

  private void publishRobotTimestamp(double timestamp) {
    telemetry.log("robotTimestamp", timestamp);
  }

  private void publishPose(Pose2d pose) {
    telemetry.log("estPose", pose);
  }

  private void publishTagCount(int tags) {
    telemetry.log("numTags", tags);
  }

  private void publishMegatag2Pose(boolean isMegatag2) {
    telemetry.log("isMegatag2Pose", isMegatag2);
  }

  private void publishValid(boolean valid) {
    telemetry.log("poseValid", valid);
  }
}
