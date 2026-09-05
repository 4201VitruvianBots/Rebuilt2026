package frc.robot.constants;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;

public class INTAKE {
  public static class ROLLERS {
    // TODO: change values
    public static final double kP = 1.0;
    public static final double gearRatio = 1.0;
    public static final double peakForwardOutput = 1.0;
    public static final double peakReverseOutput = -0.9;
    public static final double kInertia = 0.005;
    public static final double kStatorCurrentLimit = 50;

    public static final DCMotor gearbox = DCMotor.getKrakenX60(2);

    public static final Distance radius = Inches.of(0.625);

    // How much the stator current of the intake should increase when it is intaking a game piece
    // compared to when it is not.
    public static final double currentDifferenceThreshold =
        40.0; // TODO: Tune this value based on testing.

    public enum INTAKE_STATE {
      IDLE,
      INTAKING,
      SHOOTING,
      REVERSING,
      MANUAL;
    }

    public enum INTAKE_SPEED {
      ZERO(0),
      INTAKING(0.7),
      SHOOTING(0.3),
      SHOTREVERSING(0.13),
      REVERSE(-0.6);

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
    public static final double kP = 60.0;
    public static final double kD = 0.0; 
    /*so basically kS kV and kA are not being used currently so they are commented out */
    // public static final double kS = 0.0; // TODO: Calculate kS and kV as a feedforward.
    // public static final double kV = 0; // Recalc these
    // public static final double kA = 0;
    public static final double kG = 0.0;

    public static final double gearRatio = 1.0 / 43.2;
    public static final double motionMagicAcceleration = 15.0;
    public static final double motionMagicCruiseVelocity = 12.0;
    public static final double motionMagicJerk = 0.0;
    public static final double kStatorCurrentLimit = 65.0;

    public static final Angle minAngle = Degrees.of(0.0);
    public static final Angle maxAngle = Degrees.of(60.0); //old 55.0 obtainted from salahuddin barket
    public static final Angle startingAngle = maxAngle;
    public static final GravityTypeValue K_GRAVITY_TYPE_VALUE =
        GravityTypeValue
            .Arm_Cosine; /* 'tis a pivot so we use the arm one because arm cosine is for arm */
    public static final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);

    public static final Distance baseLength =
        Inches.of(15.254441); /* Completely made up and none of this is true in the slightest. */
    public static final Mass mass = Pounds.of(2); // so its actually 11

    // public static final double encoderOffset = 0.3076171875;
    public static final SensorDirectionValue encoderDirection =
        SensorDirectionValue.Clockwise_Positive;

    public enum PIVOT_SETPOINT {
      STOWED(Degrees.of(60.0)),
      INTAKING(Degrees.of(0.0)),
      JOSTLING(Degrees.of(40.0));

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
