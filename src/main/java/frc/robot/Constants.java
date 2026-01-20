// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
  }

  public class SHOOTERMOTORS {
    public static final double kP =
        0.1; // TODO: These will all need to be changed because we are attempting to reach a set rpm
    public static final double kI = 0.0;
    public static final double kD = 0.02;
    public static final double gearRatio = 1.0; // Placeholder value
    public static final double peakForwardOutput = 0.4; // Placeholder value
    public static final double peakReverseOutput = -0.35; // Placeholder value
    public static final double kInertia =
        0.01; /* This probably doesn't matter because Krakens are stupid powerful. */

    // Copied from elevator so don't you even think about testing these. No way we'd reach testing
    // phase without changing these... right?
    public static double motionMagicCruiseVelocity =
        100; // target cruise velocity of 100 rps, so 6000 rpm
    public static double motionMagicAcceleration = 40; // target acceleration of 40 rps/s

    public static double motionMagicJerk = 0.0;
    /* Make this higher to increase the acceleration.
    This increases acceleration a lot faster than increasing the MotionMagicAcceleration value.
    No jerk means trapezoidal profile. */

    public static final DCMotor gearbox = DCMotor.getKrakenX60(4);

    public enum ShooterRPM {
      IDLE(500.0),
      LOW(1000.0),
      HIGH(4000.0);

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

    public static final int kIndexerMotor1 = 50; /* TODO: change values later */
    public static final int kIndexerMotor2 = 51;
    public static final int kIndexerMotor3 = 52;

    public static final int kIntakeRollerMotor1 = 53; /*TODO: again change these values later */
    public static final int kIntakeRollerMotor2 = 54;
    public static final int kIntakePivotMotor = 55;

    public static final int kUptakeMotor = 56; /* TODO: another placeholder "Fun!" */
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

  public class INDEXERMOTORS {
    /*TODO: change every value they are ALL placeholders */
    public static final double kP = 1.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final double kS = 0.01;
    public static final double gearRatio = 1.0;
    public static final double peakForwardOutput = 0.5;
    public static final double peakReverseOutput = -0.5;
    public static final double kInertia = 0.005;

    public static final DCMotor gearbox = DCMotor.getKrakenX60(3);

    public enum INDEXERSPEED {
      ZERO(0),
      INDEXING(0.4),
      FREEING(-0.1);

      private final double value;

      INDEXERSPEED(double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }
  }
  public class INTAKEMOTORS {
    public static class ROLLERS {
      /* TODO: change values because these are ALSO placeholders yay fun */
      public static final double kP = 1.0;
      public static final double kD = 0.0;
      public static final double kV = 0.0;
      public static final double kS = 0.01;
      public static final double gearRatio = 1.0;
      public static final double peakForwardOutput = 0.5;
      public static final double peakReverseOutput = -0.5;
      public static final double kInertia = 0.005;

      public static final DCMotor gearbox = DCMotor.getKrakenX60(2);

      public enum INTAKESPEED {
        ZERO(0),
        INTAKING(0.5),
        HELPSOMETHINGSSTUCK(-0.2);

          private final double value;
          
          INTAKESPEED(double value) {
            this.value = value;
          }

          public double get() {
            return value;
          }
      }
    }
  }

  public class UPTAKEMOTORS {
    public static final double kP = 1.0; /*more placeholders FUN*/
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final double kS = 0.01;
    public static final double gearRatio = 1.0;
    public static final double peakForwardOutput = 0.5;
    public static final double peakReverseOutput = -0.5;
    public static final double kInertia = 0.005;

    public static final DCMotor gearbox = DCMotor.getKrakenX60(1);

    public enum UPTAKESPEED {
      ZERO(0),
      UPTAKING(0.6),
      WEIRDREVERSE(-0.3);

        private final double value;

        UPTAKESPEED(double value) {
          this.value = value;
        }
        public double get() {
          return value;
        }
    }
  }
}
