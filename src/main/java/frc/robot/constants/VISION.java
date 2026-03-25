// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class VISION {
  public enum CAMERA_SERVER {
    limelightL("limelight-left", "10.42.1.12"),
    limelightR("limelight-right", "10.42.1.11");

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

    private final DoublePublisher hbPub;
    private final DoublePublisher estimatedTimestamp;
    private final StructPublisher<Pose2d> estimatedPose;
    private final IntegerPublisher numTags;
    private final BooleanPublisher megatag2Pose;
    private final BooleanPublisher validPose;

    public Limelight(CAMERA_SERVER limelight) {
      this.limelight = limelight;
      var ntInst = NetworkTableInstance.getDefault();
      var llSubTable = ntInst.getTable(limelight.name);
      hbSub = llSubTable.getDoubleTopic("hb").subscribe(-1.0);

      var llPubTable = ntInst.getTable("llTable").getSubTable(limelight.name);
      hbPub = llPubTable.getDoubleTopic("heartbeat").publish();
      estimatedTimestamp = llPubTable.getDoubleTopic("estTimestamp").publish();
      estimatedTimestamp.setDefault(-1);
      estimatedPose = llPubTable.getStructTopic("estPose", Pose2d.struct).publish();
      estimatedPose.setDefault(new Pose2d(-1, -1, Rotation2d.kZero));
      numTags = llPubTable.getIntegerTopic("numTags").publish();
      numTags.setDefault(-1);
      megatag2Pose = llPubTable.getBooleanTopic("isMegatag2Pose").publish();
      megatag2Pose.setDefault(false);
      validPose = llPubTable.getBooleanTopic("poseValid").publish();
      validPose.setDefault(false);

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
      estimatedTimestamp.set(timestamp);
    }

    public void publishPose(Pose2d pose) {
      estimatedPose.set(pose);
    }

    public void publishTagCount(int tags) {
      numTags.set(tags);
    }

    public void publishMegatag2Pose(boolean isMegatag2) {
      megatag2Pose.set(isMegatag2);
    }

    public void publishValid(boolean valid) {
      validPose.set(valid);
    }

    public double getHeartbeat() {
      var heartbeat = hbSub.get();
      if (heartbeat != -1 || heartbeat != lastHeartbeat) {
        lastHeartbeat = heartbeat;
        isAlive = true;
      } else {
        isAlive = false;
      }

      hbPub.set(heartbeat);

      return lastHeartbeat;
    }

    public boolean isAlive() {
      return isAlive;
    }
  }
}
