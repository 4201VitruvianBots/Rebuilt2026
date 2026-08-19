package frc.team4201.lib.command;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import frc.team4201.lib.bline.PIDConstants;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.command2.Subsystem;
import frc.team4201.lib.vision.LimelightHelpers;

public interface SwerveSubsystem extends Subsystem {

  AngularVelocity getYawRate();

  void addVisionMeasurement(LimelightHelpers.PoseEstimate poseEstimate);

  void addVisionMeasurement(Pose2d pose, double timestampSeconds);

  void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDevs);

  /**
   * Function to get the module locations as a {@link Translation2d} array
   *
   * @return Translation2d[]
   */
  Translation2d[] getModuleLocations();

  /**
   * Function to get the translation {@link PIDConstants} for PathPlanner
   *
   * @return PIDConstants
   */
  PIDConstants getAutoTranslationPIDConstants();

  /**
   * Function to get the rotation {@link PIDConstants} for PathPlanner
   *
   * @return PIDConstants
   */
  PIDConstants getAutoRotationPIDConstants();

  /**
   * Function to get the CTRE SwerveDriveState. Used to get the Robot's {@link Pose2d}.
   *
   * @return SwerveDriveState
   */
  SwerveDriveState getState();

  /**
   * Function to reset the robot's pose given a {@link Pose2d}
   *
   * @param pose Robot's position on the field in {@link Pose2d}
   */
  void resetPose(Pose2d pose);
}
