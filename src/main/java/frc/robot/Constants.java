// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.team4201.lib.utils.ModuleMap.MODULE_POSITION;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public class SHOOTERMOTORS {
    public static final double kP =
        5.0; // TODO: These will all need to be changed because we are attempting to reach a set rpm
    public static final double kV = 0.0;
    public static final double kS = 0.0; // TODO: Calculate kS (hooo boy that's gonna be fun,
    public static final double kA = 0.0;
    // The value of kS is the largest voltage applied before the mechanism begins to move)
    public static final double gearRatio = 1.0; // Placeholder value
    public static final double peakForwardOutput = 0.4; // Placeholder value
    public static final double peakReverseOutput = -0.35; // Placeholder value
    public static final double kInertia =
        0.01; /* This probably doesn't matter because Krakens are stupid powerful. */

    // Copied from elevator so don't you even think about testing these. No way we'd reach testing
    // phase without changing these... right?
    public static double motionMagicCruiseVelocity =
        70; // target cruise velocity of 100 rps, so 6000 rpm
    public static double motionMagicAcceleration =
        60; // target acceleration of 60 rps/s.. I don't actually know if krakens can do this

    public static double motionMagicJerk = 0.0;
    /* Make this higher to increase the acceleration.
    This increases acceleration a lot faster than increasing the MotionMagicAcceleration value.
    No jerk means trapezoidal profile. */

    public static final DCMotor gearbox = DCMotor.getKrakenX60Foc(4);

    public enum ShooterRPS {
      IDLE(RotationsPerSecond.of(500.0 / 60)), // The number on the left is the RPM
      LOW(RotationsPerSecond.of(1000.0 / 60)),
      HIGH(RotationsPerSecond.of(4080.0 / 60));

      private final AngularVelocity rps;

      ShooterRPS(AngularVelocity rps) {
        this.rps = rps;
      }

      public AngularVelocity getRPS() {
        return rps;
      }
    }
  }

  public class SHOOTERHOOD {
    public static final double kP = 7.0; // TODO: Change this
    public static final double kD = 0.1;
    public static final double kA = 0.0; // TODO: Change these two feedforwards later, use ReCalc
    public static final double kV = 0.0;
    public static final double kS = 0.0;
    public static final double gearRatio = 8.0 / 1.0; // TODO: Change this later because this is confirmed not what the final thing will be
    public static final double peakForwardOutput = 0.4; // Placeholder value
    public static final double peakReverseOutput = -0.35; // Placeholder value
    public static final double kInertia =
        0.005; /* This probably doesn't matter because Krakens are stupid powerful. */

    public static final double motionMagicCruiseVelocity = 60;
    public static final double motionMagicAcceleration = 30;

    public static final Angle minAngle = Degrees.of(-10.0);
    public static final Angle maxAngle = Degrees.of(180.0);
    public static final Angle minRotations = Degrees.of(-10.0 / 360);
    public static final Angle maxRotations = Degrees.of(180.0 / 360);

    public static final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);

    public enum HoodAngle {
      // TODO: Going to stop using this because we are going to do math instead :)
      NOTHING(Degrees.of(0.0)),
      CLOSE(Degrees.of(30.0)),
      FAR(Degrees.of(60.0)),
      VERYFAR(Degrees.of(90.0));

      private final Angle angle;

      HoodAngle(Angle angle) {
        this.angle = angle;
      }

      public Angle getAngle() {
        return angle;
      }
    }
  }

  public class CAN {
    public static final String rioCanbus = "rio";
    public static String driveBaseCanbus = "drivebase";

    public static final int pigeon = 9;

    public static final int frontLeftCanCoder = 10;
    public static final int frontRightCanCoder = 11;
    public static final int backLeftCanCoder = 12;
    public static final int backRightCanCoder = 13;

    public static final int frontLeftDriveMotor = 20;
    public static final int frontLeftTurnMotor = 21;
    public static final int frontRightDriveMotor = 22;
    public static final int frontRightTurnMotor = 23;
    public static final int backLeftDriveMotor = 24;
    public static final int backLeftTurnMotor = 25;
    public static final int backRightDriveMotor = 26;
    public static final int backRightTurnMotor = 27;

    public static final int kShooterRollerMotor1 = 30;
    public static final int kShooterRollerMotor2 = 31;
    public static final int kShooterRollerMotor3 = 32;
    public static final int kShooterRollerMotor4 = 33;

    public static final int kShooterHoodMotor = 34;
  }

  // usb n swerve are like lwk copied from reefscape
  public final class USB {
    public static final int driver_xBoxController = 0;
  }

  public class SWERVE {
    // TODO: Remove unused variables
    // (maybe crossreferencing with Reefscape2025 to see what gets used in a full robot project)

    public enum MOTOR_TYPE {
      ALL,
      DRIVE,
      STEER
    }

    public static final Distance kWheelBase = Inches.of(23.75);
    public static final Distance kTrackWidth = Inches.of(23.75);
    public static final Distance kBumperThickness = Inches.of(2.5);

    public static final PIDConstants kTranslationPID = new PIDConstants(10, 0, 0);
    public static final PIDConstants kRotationPID = new PIDConstants(7, 0, 0);

    public static final Map<MODULE_POSITION, Translation2d> kModuleTranslations =
        Map.of(
            MODULE_POSITION.FRONT_LEFT,
            new Translation2d(kWheelBase.div(2).in(Meters), kTrackWidth.div(2).in(Meters)),
            MODULE_POSITION.FRONT_RIGHT,
            new Translation2d(kWheelBase.div(2).in(Meters), -kTrackWidth.div(2).in(Meters)),
            MODULE_POSITION.BACK_LEFT,
            new Translation2d(-kWheelBase.div(2).in(Meters), kTrackWidth.div(2).in(Meters)),
            MODULE_POSITION.BACK_RIGHT,
            new Translation2d(-kWheelBase.div(2).in(Meters), -kTrackWidth.div(2).in(Meters)));

    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public static final double kMaxRotationRadiansPerSecond =
        Math.PI * 0.3; // temporary to reduce speed (original value 2.0)

    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
    public static final Distance kPositionTolerance = Inches.of(0.4);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(0.25);

    public enum ROUTINE_TYPE {
      DRIVE_DYNAMIC(2),
      DRIVE_QUASISTATIC(6),
      TURN_DYNAMIC(8),
      TURN_QUASISTATIC(8);

      private final int lengthSeconds;

      ROUTINE_TYPE(int lengthSeconds) {
        this.lengthSeconds = lengthSeconds;
      }

      public int getLengthSeconds() {
        return lengthSeconds;
      }
    }
  }
}
