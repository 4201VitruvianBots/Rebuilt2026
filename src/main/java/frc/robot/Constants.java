// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.derive;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import frc.team4201.lib.utils.ModuleMap.MODULE_POSITION;
import java.util.Map;

public final class Constants {
  public class FLYWHEEL {
    public static final double kP = 9.1; // These worked for WoodBot but will need to be retuned
    public static final double kD = 0.05;
    public static final double kV = 0.0;
    public static final double kS = 2.5; // TODO: Calculate kS (hooo boy that's gonna be fun,
    public static final double kA = 0.0;
    // The value of kS is the largest voltage applied before the mechanism begins to move)
    public static final double gearRatio = 1.0; // Placeholder value
    public static final double peakForwardOutput = 0.4; // Placeholder value
    public static final double peakReverseOutput = -0.35; // Placeholder value
    public static final double kInertia = 0.01;
    public static final double kStatorCurrentLimit = 120;

    // These worked on wood bot. Change jerk later if further optimization is needed
    public static double motionMagicCruiseVelocity = 60.0; // target cruise velocity of 60 rps
    public static double motionMagicAcceleration = 30.0; // target acceleration of 30 rps/s..
    public static double motionMagicJerk = 0.0;

    public static final DCMotor gearbox =
        DCMotor.getKrakenX60Foc(1); // We have more motors than this on the final bot.

    public static class Shot {
      public final double shooterRPM;
      public final double hoodPosition;

      public Shot(double shooterRPM, double hoodPosition) {
        this.shooterRPM = shooterRPM;
        this.hoodPosition = hoodPosition;
      }
    }

    public enum MANUAL_RPM {
      IDLE(RPM.of(0.0)),
      LOW(RPM.of(1000.0)),
      HIGH(RPM.of(2900.0));

      private final AngularVelocity rpm;

      MANUAL_RPM(AngularVelocity rpm) {
        this.rpm = rpm;
      }

      public AngularVelocity getRPM() {
        return rpm;
      }
    }

    public class HOOD {
      public static final double kP = 3.0; // TODO: Change this
      public static final double kD = 0.1;
      public static final double kA = 0.0; // TODO: Change these two feedforwards later, use ReCalc
      public static final double kV = 0.0;
      public static final double kS = 0.0;
      public static final double gearRatio =
          1.0; // TODO: Change this later because this is confirmed not what the final thing
      // will be
      public static final double peakForwardOutput = 0.4; // Placeholder value
      public static final double peakReverseOutput = -0.35; // Placeholder value
      public static final double kInertia = 0.005;
      public static final double kStatorCurrentLimit = 30;

      public static final double motionMagicCruiseVelocity = 6.0;
      public static final double motionMagicAcceleration = 4.0;

      public static final Angle minAngle = Degrees.of(0.0);
      public static final Angle maxAngle = Degrees.of(45.0);

      public static final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);

      public enum MANUAL_ANGLE {
        // TODO: Going to stop using this because we are going to do math instead :)
        NOTHING(Degrees.of(0.0)),
        CLOSE(Degrees.of(30.0)),
        FAR(Degrees.of(45.0));

        private final Angle angle;

        MANUAL_ANGLE(Angle angle) {
          this.angle = angle;
        }

        public Angle getAngle() {
          return angle;
        }
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

    public static final int kClimberMotor = 30;

    public static final int kShooterRollerMotor1 = 40;
    public static final int kShooterRollerMotor2 = 41;
    public static final int kShooterRollerMotor3 = 42;
    public static final int kShooterRollerMotor4 = 43;

    public static final int kIndexerMotor1 = 50;
    public static final int kIndexerMotor2 = 51;
    public static final int kIndexerMotor3 = 52;

    public static final int kShooterHoodMotor = 34;
    public static final int kShooterHoodCANCoder = 35;

    public static final int kIntakeRollerMotor1 = 53;
    public static final int kIntakeRollerMotor2 = 54;

    public static final int kIntakePivotMotor = 55;
    public static final int kPivotEncoder = 56;

    public static final int kUptakeMotor = 57;
  }

  public final class USB {
    public static final int driver_xBoxController = 0;
  }
  
  public final class PWM {
    public static final int kLED = 0;
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
    public static final Distance upperLimit = Inches.of(35.0); // TODO: Talk to design about height.
    public static final Distance lowerLimit = Inches.of(0.0);

    public static final Distance cruiseVelocityInches = Inches.of(14.57);

    public static final double gearRatio = 43.2 / 1.0; // Climber Gear ratio.
    public static final Distance kClimberDrumDiameter =
        Inches.of(
            2.211024); // TODO: Find what this year's climber constants are compared to last year's
    // elevator. It should just be the gear ratio that is different from last year.
    public static final Distance drumRotationsToDistance = kClimberDrumDiameter.times(Math.PI);
    public static final GravityTypeValue K_GRAVITY_TYPE_VALUE = GravityTypeValue.Elevator_Static;
    public static final Current kStatorCurrentLimit = Amps.of(40);

    public class NOT_HOLDING_ROBOT {
      public static int slot = 0;
      public static double kS = 1.0;
      // Config for Motor. TODO: Change placeholder values later.
      // public static double kV =
      //     0.0; // Placeholder. Output per unit of target velocity (output/rps).
      // public static double kA =
      //     0.0; // Placeholder. Output per unit of target acceleration (output/(rps/s)).
      public static double kP =
          200.0; // Placeholder. Output per unit of error in position (output/rotation).
      // public static double kD =
      //     0.0; // Placeholder. Output per unit of error in velocity (output/rps).
      public static double kG = 5.0;

      public static double motionMagicCruiseVelocity =
          cruiseVelocityInches.in(Meters)
              / CLIMBER.drumRotationsToDistance.in(Meters); // Placeholder.
      public static double motionMagicAcceleration = 30; // Placeholder.
      public static double motionMagicJerk = 0.0; // Placeholder.

      public static Mass kUnweightedCarriageMass = Pound.of(15); // TODO: Change this later.
    }

    // This needs to be here because of the friction of the carriage... meaning the whole
    // robot
    public class HOLDING_ROBOT {
      public static int slot = 1;
      public static double kS = 1.0;
      // public static double kV = 0.0; // For lifting the robot as well.
      // public static double kA = 0.0;
      public static double kP = 150.0;
      // public static double kD = 0.0;
      public static double kG = 5.0;

      public static Mass kWeightedCarriageMass = Pound.of(150); // TODO: Change this later.

      public static double motionMagicCruiseVelocity = 20; // Placeholder.
      public static double motionMagicAcceleration = 30;
      public static double motionMagicJerk = 0.0; // Placeholder.
    }

    public static final double peakForwardOutput = 1.00; // Placeholder.
    public static final double peakReverseOutput = -0.5; // Placeholder.

    public static final Current kHoldingRobotThreshold =
        Amps.of(15.0); // Amount of current you must be measuring to be sure that you are carrying a
    // robot

    public static DCMotor gearbox = DCMotor.getKrakenX60Foc(1);

    public enum CLIMBER_SETPOINT {
      START_POSITION(Inches.of(0.0)),
      LEVEL_ONE(Inches.of(22.0)),
      LEVEL_TWO_AND_THREE(Inches.of(17));

      private final Distance setpoint;

      CLIMBER_SETPOINT(Distance setpoint) {
        this.setpoint = setpoint;
      }

      public Distance getSetpoint() {
        return setpoint;
      }
    }
  }

  public class INDEXER {
    // TODO: change values
    public static final double kP = 1.0;
    public static final double gearRatio = 1.0;
    public static final double peakForwardOutput = 0.8;
    public static final double peakReverseOutput = -0.5;
    public static final double kInertia = 0.005;

    public static final DCMotor gearbox = DCMotor.getKrakenX60(3);

    public enum INDEXER_SPEED {
      ZERO(0),
      INDEXING(0.9),
      FREEING(-0.1);

      private final double value;

      INDEXER_SPEED(double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }
  }

  public class INTAKE {
    public static class ROLLERS {
      // TODO: change values
      public static final double kP = 1.0;
      public static final double gearRatio = 1.0;
      public static final double peakForwardOutput = 0.5;
      public static final double peakReverseOutput = -0.5;
      public static final double kInertia = 0.005;

      public static final DCMotor gearbox = DCMotor.getKrakenX60(2);

      public enum INTAKE_SPEED {
        ZERO(0),
        INTAKING(0.55),
        REVERSE(-0.2);

        private final double value;

        INTAKE_SPEED(double value) {
          this.value = value;
        }

        public double get() {
          return value;
        }
      }
    }

    public static class PIVOT {
      /* TODO: change any more values yay placeholders FUN FUN FUN HAPPY */
      public static final double kP = 1000.0;
      public static final double kD =
          100.0; /*so basically kS kV and kA are not being used currently so they are commented out */
      // public static final double kS = 0.0; // TODO: Calculate kS and kV as a feedforward.
      // public static final double kV = 0; // Recalc these
      // public static final double kA = 0;

      public static final double gearRatio = 1.0; // encoder is after gear ratio
      public static final double motionMagicAcceleration = 35.0;
      public static final double motionMagicCruiseVelocity = 25.0;
      public static final double motionMagicJerk = 0.0;

      public static final Angle minAngle = Degrees.of(0.0);
      public static final Angle maxAngle = Degrees.of(110.0);
      public static final Angle startingAngle = minAngle;
      public static final GravityTypeValue K_GRAVITY_TYPE_VALUE =
          GravityTypeValue
              .Arm_Cosine; /* 'tis a pivot so we use the arm one because arm cosine is for arm */
      public static final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);

      public static final Distance baseLength =
          Inches.of(13.897040); /* Almost completely made up :P */
      public static final Mass mass = Pounds.of(2); // TODO: Consult CAD

      public static final double encoderOffset = 0.0;
      public static final SensorDirectionValue encoderDirection =
          SensorDirectionValue.CounterClockwise_Positive;

      public enum PIVOT_SETPOINT {
        STOWED(Degrees.of(0.0)),
        INTAKING(Degrees.of(90.0));

        private final Angle angle;

        PIVOT_SETPOINT(Angle angle) {
          this.angle = angle;
        }

        public Angle getAngle() {
          return angle;
        }
      }
    }
  }

  public class UPTAKE {
    public static final double kP = 10.1; // Placeholders
    public static final double kV = 0.0;
    public static final double kS = 0.0;
    public static final double gearRatio = 1.0;
    public static final double peakForwardOutput = 0.5;
    public static final double peakReverseOutput = -0.5;
    public static final double kInertia = 0.01;

    public static final double kMotionMagicAcceleration = 30.0;
    public static final double kMotionMagicCruiseVelocity = 60.0;

    public static final AngularVelocity minRPM = RPM.of(0.0);
    public static final AngularVelocity maxRPM = RPM.of(5000.0);

    public static final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);

    public enum UPTAKE_SPEED {
      IDLE(RPM.of(0.0)),
      UPTAKING(RPM.of(500.0));

      private final AngularVelocity value;

      UPTAKE_SPEED(AngularVelocity value) {
        this.value = value;
      }

      public AngularVelocity get() {
        return value;
      }
    }
  }

  public class LED {
    public static final int numLEDs = 60; // Placeholder value, change to actual number of LEDs on the bot
    
    public enum LED_STATES {
      DISABLED("disabled"),
      IDLE("idle"),
      DRIVING("driving"),
      INTAKING("intaking"),
      SHOOTING("shooting"),
      CLIMBING("climbing");

      private final String animation;

      LED_STATES(String animation) {
        this.animation = animation;
      }

      public String getAnimation() {
        return animation;
      }
    }
  }

  public class SIM {
    // For some reason, the line width value needs to be 2 times larger (~12 times larger in Glass)
    // in order to actually match up with the visual thickness of lines in the Mechanism2d
    public static final DistanceUnit LineWidthInches =
        derive(Inches).splitInto(2).named("LineWidthInches").symbol("lw in").make();
  }
}
