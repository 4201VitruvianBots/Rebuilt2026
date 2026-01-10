package frc.team4201.lib.command;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.team4201.lib.vision.LimelightHelpers;

public interface SwerveSubsystem {

  AngularVelocity getYawRate();

  void addVisionMeasurement(LimelightHelpers.PoseEstimate poseEstimate);

  void addVisionMeasurement(Pose2d pose, double timestampSeconds);

  void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDevs);
}
