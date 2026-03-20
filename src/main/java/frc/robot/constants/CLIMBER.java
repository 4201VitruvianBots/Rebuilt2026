package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class CLIMBER {
  public static final Distance upperLimit = Inches.of(35.0); // TODO: Talk to design about height.
  public static final Distance lowerLimit = Inches.zero();

  public static final Distance cruiseVelocityInches = Inches.of(14.57);

  public static final double gearRatio = 43.2 / 1.0; // Climber Gear ratio.
  public static final Distance kClimberDrumDiameter =
      Inches.of(
          2.211024); // TODO: Find what this year's climber constants are compared to last year's
  // elevator. It should just be the gear ratio that is different from last year.
  // TODO: Simplify the conversion
  public static final Per<DistanceUnit, AngleUnit> drumRotationsToDistance =
      kClimberDrumDiameter.times(gearRatio).div(Radians.of(Math.PI));
  public static final Per<LinearVelocityUnit, AngularVelocityUnit> drumRpsToMps =
      kClimberDrumDiameter.times(gearRatio).div(Second.one()).div(RotationsPerSecond.of(Math.PI));
  public static final Per<AngleUnit, DistanceUnit> distanceToDrumRotations =
      Radians.of(Math.PI).div(kClimberDrumDiameter.times(gearRatio));
  public static final Per<AngularVelocityUnit, LinearVelocityUnit> drumMpsToRps =
      RotationsPerSecond.of(Math.PI).div(kClimberDrumDiameter.times(gearRatio).div(Second.one()));

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

    // TODO: Switch to Java Units for Conversion
    public static double motionMagicCruiseVelocity =
        cruiseVelocityInches
            .timesConversionFactor(distanceToDrumRotations)
            .in(Rotations); // Placeholder.
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
    START_POSITION(Inches.zero()),
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
