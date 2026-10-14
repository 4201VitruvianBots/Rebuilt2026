// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.net.PortForwarder;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.IntegerPublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

public final class VISION {
  public enum CAMERA_SERVER {
    limelightF("limelight-front");

    private final String name;

    CAMERA_SERVER(final String name) {
      this.name = name;
    }

    @Override
    public String toString() {
      return name;
    }
  }

  public static final Angle kLimelight4HFOV = Degrees.of(82.0);
  public static final Angle kLimelight4VFOV = Degrees.of(56.2);
  public static final Angle kLimelight4DFOV = Degrees.of(75.07);

  // TODO: Update values
  // Camera offset from robot center. Camera F is positioned on the hopper
  public static final Transform3d limelightFPosition =
      new Transform3d(
          new Translation3d(
              Meters.of(0).magnitude(), Meters.of(0).magnitude(), Meters.of(0).magnitude()),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(180)));

  // TODO: Update values
  // Camera offset from robot center. Camera B is facing out of the rear of the robot (On the
  // EndEffector side)
  public static final Transform3d limelightBPosition =
      new Transform3d(
          new Translation3d(
              Meters.of(0).magnitude(), Meters.of(0).magnitude(), Meters.of(0).magnitude()),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(180)));

  public static final Distance poseXTolerance = Inches.of(4);
  public static final Distance poseYTolerance = Inches.of(4);
  public static final Distance poseZTolerance = Inches.of(4);
  public static final Angle posePitchTolerance = Degrees.of(4);
  public static final Angle poseRollTolerance = Degrees.of(4);
  public static final Angle poseYawTolerance = Degrees.of(4);

  public enum TARGET {
    LEFT_FRONT_TOWER,
    RIGHT_FRONT_TOWER,
    // LEFT_BACK_TOWER, // add if needed
    // RIGHT_BACK_TOWER // add if needed
  }
}
