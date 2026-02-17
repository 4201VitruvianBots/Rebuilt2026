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

  public enum INDEXER_SPEED_1 {
    ZERO(0),
    INDEXING(0.9),
    FREEING(-0.3);

    private final double value;

    INDEXER_SPEED_1(double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public enum INDEXER_SPEED_2 {
    ZERO(0),
    INDEXING(0.9),
    FREEING(-0.3);

    private final double value;

    INDEXER_SPEED_2(double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }
}
