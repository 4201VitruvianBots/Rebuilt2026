// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pound;
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
import edu.wpi.first.units.measure.Mass;
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
    // TODO: These will all need to be changed because we are attempting to reach a set rpm.
    public static final double kP = 5.0; 
    public static final double kV = 0.0;
    public static final double kS = 0.0; // TODO: Calculate kS (hooo boy that's gonna be fun.
    public static final double kA = 0.0;
    // The value of kS is the largest voltage applied before the mechanism begins to move.
    public static final double gearRatio = 1.0; // Placeholder value.
    public static final double peakForwardOutput = 0.4; // Placeholder value.
    public static final double peakReverseOutput = -0.35; // Placeholder value.
    public static final double kInertia =
        0.01; /* This probably doesn't matter because Krakens are stupid powerful. */

    // These worked on wood bot. Change jerk later if further optimization is needed.
    public static double motionMagicCruiseVelocity =
        35.0; // Target cruise velocity of 35 rps, so 2100 rpm.
    public static double motionMagicAcceleration = 22.0; // Target acceleration of 22 rps/s.
    public static double motionMagicJerk = 0.0;

    public static final DCMotor gearbox = DCMotor.getKrakenX60Foc(4);

    public enum ShooterVelocity {
      IDLE(0.0),
      LOW(1000.0),
      HIGH(2900.0);

      private final double rpm;

      ShooterVelocity(double rpm) {
        this.rpm = rpm;
      }

      public AngularVelocity getRPM() {
        return RotationsPerSecond.of(rpm / 60.0);
      }
    }
  }

  public class SHOOTERHOOD {
    public static final double kP = 3.0; // TODO: Change this.
    public static final double kD = 0.1;
    public static final double kA = 0.0; // TODO: Change these two feedforwards later, use ReCalc.
    public static final double kV = 0.0;
    public static final double kS = 0.0;
    public static final double gearRatio =
        1.0; // TODO: Change this later because this is confirmed not what the final thing will be.
    public static final double peakForwardOutput = 0.4; // Placeholder value.
    public static final double peakReverseOutput = -0.35; // Placeholder value.
    public static final double kInertia =
        0.005; /* This probably doesn't matter because Krakens are stupid powerful. */

    public static final double motionMagicCruiseVelocity = 6.0;
    public static final double motionMagicAcceleration = 4.0;

    public static final Angle minAngle = Degrees.of(0.0);
    public static final Angle maxAngle = Degrees.of(45.0);

    public static final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);

    public enum HoodAngle {
      // TODO: Going to stop using this because we are going to do math instead :)
      NOTHING(Degrees.of(0.0)),
      CLOSE(Degrees.of(30.0)),
      FAR(Degrees.of(45.0));

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
    public static final String driveBaseCanbus = "drivebase";

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

    public static final int kClimberMotor = 30; // TODO: Change this later.

    public static final int kShooterRollerMotor1 = 40;
    public static final int kShooterRollerMotor2 = 41;
    public static final int kShooterRollerMotor3 = 42;
    public static final int kShooterRollerMotor4 = 43;

    public static final int kIndexerMotor1 = 50; /* TODO: Change values later. */
    public static final int kIndexerMotor2 = 51;
    public static final int kIndexerMotor3 = 52;

    public static final int kShooterHoodMotor = 34;
    public static final int kShooterHoodCANCoder = 35;
    public static final int kIntakeRollerMotor1 = 53; /*TODO: Again change these values later. */
    public static final int kIntakeRollerMotor2 = 54;
    public static final int kIntakePivotMotor = 55;

    public static final int kUptakeMotor = 56; /* TODO: Another placeholder "Fun!" */
  }

  // Usb n swerve are like lwk copied from reefscape.
  public final class USB {
    public static final int driver_xBoxController = 0;
  }

  public class SWERVE {
    // TODO: Remove unused variables.
    // (maybe crossreferencing with Reefscape2025 to see what gets used in a full robot project).

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
        Math.PI * 0.3; // Temporary to reduce speed (original value 2.0).

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

  public class CLIMBER {
    public static final Distance upperLimit = Inches.of(20); // TODO: Talk to design about height.
    public static final Distance lowerLimit = Inches.of(0);

    // Config for Motor. TODO: Change placeholder values later.
    public static double kV = 1.0; // Placeholder. Output per unit of target velocity (output/rps).
    public static double kA = 1.0; // Placeholder. Output per unit of target acceleration (output/(rps/s)).
    public static double kP = 1.0; // Placeholder. Output per unit of error in position (output/rotation).
    public static double kD = 1.0; // Placeholder. Output per unit of error in velocity (output/rps).

    public static final double gearRatio = 1.0 / 27.0; // Climber Gear ratio.

    public static double motionMagicCruiseVelocity = 20; // Placeholder.
    public static double motionMagicAcceleration = 30; // Placeholder.

    public static final double peakForwardOutput = 1.00; // Placeholder.
    public static final double peakReverseOutput = -0.5; // Placeholder.

    public static DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    public static Mass kCarriageMass = Pound.of(15); // TODO: Change this later.
        /* Carriage mass will be the weight that the "carriage,"" in this case climber, is holding.
        It might be dynamic, because you're lifting the weight of the elevator, 
        but you also might be lifting the weight of the robot. */
    public static final Distance kClimberDrumDiameter = Inches.of(2.2557); // Placeholder.
  }

  public class ROBOT {

    // Climber control mode:
    public enum CONTROL_MODE {
      OPEN_LOOP,
      CLOSED_LOOP
    }
  }

  public class INDEXERMOTORS {
    /* TODO: Change every value they are ALL placeholders. */
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
      /* TODO: Change values because these are ALSO placeholders yay fun. */
      public static final double kP = 1.0;
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
    public static final double kP = 1.0; /* TODO: More placeholders FUN. */
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
