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
    limelightL("limelight-left", "10.42.1.13"),
    limelightR("limelight-right", "10.42.1.11"),
    limelightF("limelight-front", "10.42.1.12");

    private final String name;
    private final String ip;
    private final int lastOctet;

    CAMERA_SERVER(final String name, final String ip) {
      this.name = name;
      this.ip = ip;
      this.lastOctet = Integer.parseInt(this.ip.substring(this.ip.length() - 2));
    }

    public String getIp() {
      return ip;
    }

    public int getLastOctet() {
      return lastOctet;
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

  public static class Limelight {
    static int basePort = 5800;
    CAMERA_SERVER limelight;
    private final DoubleSubscriber hbSub;
    private double lastHeartbeat = -1.0;
    private boolean isAlive = false;

    private final TelemetryTable telemetery;

    public Limelight(CAMERA_SERVER limelight) {
      this.limelight = limelight;
      var ntInst = NetworkTableInstance.getDefault();
      var llSubTable = ntInst.getTable(limelight.name);
      hbSub = llSubTable.getDoubleTopic("hb").subscribe(-1.0);

      telemetery = Telemetry.getTable("llTable").getTable(limelight.name);

      for (int i = 0; i < 10; i++) {
        int ethPort = basePort + i;
        int usbPort = ethPort + (limelight.ordinal() * 10);
        PortForwarder.add(usbPort, this.limelight.ip, ethPort);
      }
    }

    public String getName() {
      return limelight.name;
    }

    public void publishTimestamp(double timestamp) {
      telemetery.log("estTimestamp", timestamp);
    }

    public void publishRobotTimestamp(double timestamp) {
      telemetery.log("robotTimestamp", timestamp);
    }

    public void publishPose(Pose2d pose) {
      telemetery.log("estPose", pose);
    }

    public void publishTagCount(int tags) {
      telemetery.log("numTags", tags);
    }

    public void publishMegatag2Pose(boolean isMegatag2) {
      telemetery.log("isMegatag2Pose", isMegatag2);
    }

    public void publishValid(boolean valid) {
      telemetery.log("poseValid", valid);
    }

    public double getHeartbeat() {
      var heartbeat = hbSub.get();
      if (heartbeat != -1 || heartbeat != lastHeartbeat) {
        lastHeartbeat = heartbeat;
        isAlive = true;
      } else {
        isAlive = false;
      }

      telemetery.log("hb", heartbeat);

      return lastHeartbeat;
    }

    public boolean isAlive() {
      return isAlive;
    }
  }
}
