package frc.robot.constants;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.RPM;

import org.wpilib.math.system.DCMotor;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

public class FLYWHEEL {
  public static final double kP = 19.1; // These worked for WoodBot but will need to be retuned
  public static final double kA = 0.0;
  // The value of kS is the largest voltage applied before the mechanism begins to move)
  public static final double gearRatio = 32.0 / 24.0; // Placeholder value
  public static final double kInertia = 0.01;
  public static final double kStatorCurrentLimit = 70.0;
  public static final double kVelocityErrorThresholdTeleop = 100.0;
  public static final double kVelocityErrorThresholdAuto = 150.0;
  public static final double kFuelDragCoefficient =
      0.48; // Estimation based on it's size and relatively smooth shape. TODO: Tune
  public static final double kRumbleStrength = 0.25;

  // These worked on wood bot. Change jerk later if further optimization is needed
  public static double motionMagicCruiseVelocity = 60.0; // target cruise velocity of 60 rps
  public static double motionMagicAcceleration = 30.0; // target acceleration of 30 rps/s..
  public static double motionMagicJerk = 0.0;

  public static final DCMotor gearbox =
      DCMotor.getKrakenX60Foc(3); // We have more motors than this on the final bot.

  public static final Distance fuelLaunchHeight = Inches.of(26.15);
  public static final Distance radius = Inches.of(2.0);

  public static final int ballsPerSecond = 18;
  public static final double defaultFireDurationSeconds = 2.7;

  public static final AngularVelocity rpmShiftIncrement = RPM.of(10.0);

  public static class Shot {
    public final AngularVelocity shooterRPM;
    public final Angle hoodAngle;
    public final double timeOfFlight;

    public Shot(AngularVelocity shooterRPM, Angle hoodAngle, double timeOfFlight) {
      this.shooterRPM = shooterRPM;
      this.timeOfFlight = timeOfFlight;
      this.hoodAngle = hoodAngle;
    }
  }

  public enum MANUAL_RPM {
    IDLE(RPM.of(0.0)),
    HUB(RPM.of(1260.0 - 10)), // Old value from v1: 1470
    BUMP(RPM.of(1678.683948)), //Calculated using sim (Citrus Refrence????)
    TOWER(RPM.of(1719.0)),
    PASSING(RPM.of(2300.0)),
    FULL(RPM.of(3800.0));

    private final AngularVelocity rpm;

    MANUAL_RPM(AngularVelocity rpm) {
      this.rpm = rpm;
    }

    public AngularVelocity getRPM() {
      return rpm;
    }
  }

  public class HOOD {
    public static final double kP = 230; // TODO: Change this
    public static final double kS = 0.349609375;
    public static final double gearRatio = 170.0 / 10.0;
    public static final double rotorToSensorRatio = 3.111;
    public static final double kInertia = 0.005;
    public static final double kStatorCurrentLimit = 15;
    public static final SensorDirectionValue K_SENSOR_DIRECTION_VALUE =
        SensorDirectionValue.Clockwise_Positive;
    public static final double kMagnetSensorOffset = -0.114013671875;
    public static final double kAbsoluteSensorDiscontinuityPoint = 0.85;

    public static final double motionMagicCruiseVelocity = 68.0;
    public static final double motionMagicAcceleration = 64.0;
    public static final double motionMagicJerk = 1500.0;

    public static final Angle minAngle = Degrees.of(0.0);
    public static final Angle maxAngle = Degrees.of(19.0);

    public static final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);

    public static final Angle angleShiftIncrement = Degrees.of(0.25);

    public enum MANUAL_ANGLE {
      STOWED(Degrees.of(0.0)),
      HUB(Degrees.of(0.0)), // Old value from v1: 1.0
      BUMP(Degrees.of(4.570313)), //calculated using sim
      TOWER(Degrees.of(8.3)),
      PASSING(Degrees.of(17.5)),
      FULL(Degrees.of(19.0));


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
