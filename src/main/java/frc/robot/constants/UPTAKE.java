package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;

public class UPTAKE {
  public static final double kP = 13.5; // Placeholders
  public static final double kV = 0.0;
  public static final double kS = 24.3603515625;
  public static final double gearRatio = 1.0;
  public static final double kInertia = 0.01;
  public static final double kVelocityErrorThreshold = 35.0;

  public static final double kMotionMagicAcceleration = 30.0;
  public static final double kMotionMagicCruiseVelocity = 60.0;

  public static final AngularVelocity minRPM =
      RPM.of(0.0); // Restrict uptake velocity harder than the flywheel
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
