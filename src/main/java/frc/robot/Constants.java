// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
  }

  public class SHOOTERMOTORS {
    public static final double kP =
        0.1; // TODO: These will all need to be changed because we are attempting to reach a set rpm
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double gearRatio = 1.0; // Placeholder value
    public static final double peakForwardOutput = 1.0; // Placeholder value
    public static final double peakReverseOutput = -1.0; // Placeholder value
    public static final double kInertia =
        0.0001; /* This probably doesn't matter because Krakens are stupid powerful. */

    // Copied from elevator so don't you even think about testing these. No way we'd reach testing
    // phase without changing these... right?
    public static double motionMagicCruiseVelocity =
        100; // target cruise velocity of 100 rps, so 6000 rpm
    public static double motionMagicAcceleration = 40; // target acceleration of 40 rps/s

    public static double motionMagicJerk = 0;
    /* Make this higher to increase the acceleration.
    This increases acceleration a lot faster than increasing the MotionMagicAcceleration value.
    No jerk means trapezoidal profile. */

    public static final DCMotor gearbox = DCMotor.getKrakenX60(4);

    public enum ShooterRPM {
      IDLE(500.0),
      LOW(1000.0),
      HIGH(6000.0);

      private final double rpm;

      ShooterRPM(double rpm) {
        this.rpm = rpm;
      }

      public double getRPM() {
        return rpm;
      }
    }
  }

  public class CAN {
    public static final int kShooterRollerMotor1 = 30;
    public static final int kShooterRollerMotor2 = 31;
    public static final int kShooterRollerMotor3 = 32;
    public static final int kShooterRollerMotor4 = 33;
  }
}
