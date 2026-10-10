package frc.robot.constants;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.RPM;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

public class FLYWHEEL {
  public static final double kP = 14.0; // These worked for WoodBot but will need to be retuned
  public static final double kV = 12.0 / 96.66666667;
  // The value of kS is the largest voltage applied before the mechanism begins to move)
  public static final double gearRatio = 24.0 / 36.0; // Placeholder value
  public static final double kInertia = 0.01;
  public static final double kStatorCurrentLimit = 60.0;
  public static final double kSupplyCurrentLimit = 60.0;
  public static final double kVelocityErrorThresholdTeleop = 100.0;
  public static final double kVelocityErrorThresholdAuto = 150.0;
  public static final double kFuelDragCoefficient =
      0.48; // Estimation based on it's size and relatively smooth shape. TODO: Tune
  public static final double kRumbleStrength = 0.25;
  public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;

  // These worked on wood bot. Change jerk later if further optimization is needed
  public static double motionMagicCruiseVelocity = 60.0; // target cruise velocity of 60 rps
  public static double motionMagicAcceleration = 30.0; // target acceleration of 30 rps/s..
  public static double motionMagicJerk = 0.0;

  public static final DCMotor gearbox =
      DCMotor.getKrakenX60Foc(4); // We have more motors than this on the final bot.

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
    HUB(RPM.of(4112)), // Old value from v1: 1470
    BUMP(RPM.of(1678.683948)), // Calculated using sim (Citrus Refrence????)
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
    public static final double kP = 190.0; // TODO: Change this
    public static final double kS = 0.28;
    public static final double gearRatio = 62.4 / 1.0;
    public static final double kInertia = 0.005;
    public static final double kStatorCurrentLimit = 40;
    public static final SensorDirectionValue K_SENSOR_DIRECTION_VALUE =
        SensorDirectionValue.Clockwise_Positive;
    public static final double kMagnetSensorOffset = -0.114013671875;
    public static final double kAbsoluteSensorDiscontinuityPoint = 0.85;

    public static final double motionMagicCruiseVelocity = 10.0;
    public static final double motionMagicAcceleration = 7.0;
    public static final double motionMagicJerk = 1500.0; // Yk What'd be fun, graphing this 

    public static final Angle minAngle = Degrees.of(0.0);
    public static final Angle maxAngle = Degrees.of(90.0); //something like that

    public static final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);

    public static final Angle angleShiftIncrement = Degrees.of(0.25);
    //I feel like this would be a way to implement it? Value is 100% made up
    public static final Angle hoodReverseOffset = Degrees.of(90);

    public enum MANUAL_ANGLE {
      STOWED(Degrees.of(2.0)),
      HUB(Degrees.of(90.0)), // Test to highest possible angle with valid backwards shot
      BUMP(Degrees.of(30.0)), // calculated using sim
      TOWER(Degrees.of(20.0)),
      REVERSE(Degrees.of(80.0)), 
      PASSING(Degrees.of(45.0)),
      FULL(Degrees.of(90.0));

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
