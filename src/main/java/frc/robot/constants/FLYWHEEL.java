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
  public static final double kP = 11.5; // These worked for WoodBot but will need to be retuned
  public static final double kD = 0.02;
  ;
  public static final double kV = 0.0;
  public static final double kS = 0.0; // TODO: Calculate kS (hooo boy that's gonna be fun,
  public static final double kA = 0.0;
  // The value of kS is the largest voltage applied before the mechanism begins to move)
  public static final double gearRatio = 32.0 / 24.0; // Placeholder value
  public static final double kInertia = 0.01;
  public static final double kStatorCurrentLimit = 70.0;
  public static final double kVelocityErrorThreshold = 150.0;

  // These worked on wood bot. Change jerk later if further optimization is needed
  public static double motionMagicCruiseVelocity = 60.0; // target cruise velocity of 60 rps
  public static double motionMagicAcceleration = 30.0; // target acceleration of 30 rps/s..
  public static double motionMagicJerk = 0.0;

  public static final DCMotor gearbox =
      DCMotor.getKrakenX60Foc(3); // We have more motors than this on the final bot.

  public static final Distance fuelLaunchHeight = Inches.of(26.15);
  public static final Distance radius = Inches.of(2.0);

  public static final int ballsPerSecond = 15;

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
    public static final double kP = 12.1; // TODO: Change this
    public static final double kS = 0.47;
    public static final double gearRatio =
        170.0 / 10.0; // TODO: Change this later because this is confirmed not what the final thing
    // will be
    public static final double kInertia = 0.005;
    public static final double kStatorCurrentLimit = 80;
    public static final SensorDirectionValue K_SENSOR_DIRECTION_VALUE =
        SensorDirectionValue.Clockwise_Positive;
    public static final double kMagnetSensorOffset = -0.31591796875;
    public static final double kAbsoluteSensorDiscontinuityPoint = 1.0;

    public static final double motionMagicCruiseVelocity = 22.0;
    public static final double motionMagicAcceleration = 14.0;

    public static final Angle minAngle = Degrees.of(0.0);
    public static final Angle maxAngle = Degrees.of(25.5);

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
