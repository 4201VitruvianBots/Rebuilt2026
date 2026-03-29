package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class FLYWHEEL {
  public static final double kP = 19.100000381469727; // These worked for WoodBot but will need to be retuned 
  public static final double kA = 0.0;
  // The value of kS is the largest voltage applied before the mechanism begins to move)
  public static final double gearRatio = 32.0 / 24.0; // Placeholder value
  public static final double kInertia = 0.01;
  public static final double kStatorCurrentLimit = 70.0;
  public static final double kVelocityErrorThreshold = 100.0;
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
    HUB(RPM.of(1470.0)),
    TOWER(RPM.of(1719.0)),
    PASSING(RPM.of(2300.0));

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
    public static final double gearRatio =
        170.0 / 10.0; 
    public static final double rotorToSensorRatio = 3.111;
    public static final double kInertia = 0.005;
    public static final double kStatorCurrentLimit = 15;
    public static final SensorDirectionValue K_SENSOR_DIRECTION_VALUE =
        SensorDirectionValue.Clockwise_Positive;
    public static final double kMagnetSensorOffset = 0.245361328125;
    public static final double kAbsoluteSensorDiscontinuityPoint = 0.85;

    public static final double motionMagicCruiseVelocity = 68.0;
    public static final double motionMagicAcceleration = 64.0;
    public static final double motionMagicJerk = 1500.0;

    public static final Angle minAngle = Degrees.of(0.0);
    public static final Angle maxAngle = Degrees.of(19.0);

    public static final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);

    public enum MANUAL_ANGLE {
      STOWED(Degrees.of(0.0)),
      HUB(Degrees.of(1.0)),
      TOWER(Degrees.of(8.3)),
      PASSING(Degrees.of(17.5));

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
