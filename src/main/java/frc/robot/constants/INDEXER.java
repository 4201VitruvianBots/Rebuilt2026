package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class INDEXER {
  // TODO: change values
  public static final double kP = 1.0;
  public static final double gearRatio = 1.0;
  public static final double peakForwardOutput = 0.9;
  public static final double peakReverseOutput = -0.9;
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
