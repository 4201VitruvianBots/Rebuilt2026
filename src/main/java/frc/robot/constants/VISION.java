// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class VISION {
  public enum CAMERA_SERVER {
    limelightR("limelight-right", "10.42.1.11"),
    limelightL("limelight-left", "10.42.1.12");

    private final String name;
    private final String ip;

    CAMERA_SERVER(final String name, final String ip) {
      this.name = name;
      this.ip = ip;
    }

    public String getIp() {
      return ip;
    }

    @Override
    public String toString() {
      return name;
    }
  }

  public static final Distance poseXTolerance = Inches.of(4);
  public static final Distance poseYTolerance = Inches.of(4);
  public static final Distance poseZTolerance = Inches.of(4);
  public static final Angle posePitchTolerance = Degrees.of(4);
  public static final Angle poseRollTolerance = Degrees.of(4);
  public static final Angle poseYawTolerance = Degrees.of(4);
}
