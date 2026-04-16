package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

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
      INTAKING(0.95),
      AUTOINTAKING(0.99),
      SHOOTING(0.9),
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
    public static final double kP = 31.0;
    public static final double kI = 35;
    public static final double kD =
        0.0; /*so basically kS kV and kA are not being used currently so they are commented out */
    // public static final double kS = 0.0; // TODO: Calculate kS and kV as a feedforward.
    // public static final double kV = 0; // Recalc these
    // public static final double kA = 0;
    public static final double kG = 0.85;

    public static final double gearRatio = 175.0 / 9.0; // encoder is after gear ratio
    public static final double SensorToMechanismRatio = 3.0 / 1.0;
    public static final double motionMagicAcceleration = 15.0;
    public static final double motionMagicCruiseVelocity = 12.0;
    public static final double motionMagicJerk = 0.0;
    public static final double kStatorCurrentLimit = 65.0;

    public static final Angle minAngle = Degrees.of(0.0);
    public static final Angle maxAngle = Degrees.of(105.0);
    public static final Angle startingAngle = maxAngle;
    public static final GravityTypeValue K_GRAVITY_TYPE_VALUE =
        GravityTypeValue
            .Arm_Cosine; /* 'tis a pivot so we use the arm one because arm cosine is for arm */
    public static final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);

    public static final Distance baseLength =
        Inches.of(13.897040); /* Almost completely made up :P */
    public static final Mass mass = Pounds.of(2); // TODO: Consult CAD

    public static final double encoderOffset = -0.7646484375;
    public static final SensorDirectionValue encoderDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    public static final double kAbsoluteSensorDiscontinuityPoint = 1;

    public static final Time pivotCycleTime = Seconds.of(0.5); // original: 1.65

    public enum PIVOT_SETPOINT {
      STOWED(Degrees.of(0.0)),
      INTAKING(Degrees.of(102.421875)),
      JOSTLING(Degrees.of(20.0)),
      DEFUEL(Degrees.of(48.053));

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
