package frc.team4201.lib.bline;

public record PIDConstants(double iZone, double kD, double kI, double kP) {
  public PIDConstants(double kP, double kI, double kD) {
    this(kP, kI, kD, 1.0);
  }

  public PIDConstants(double kP, double kD) {
    this(kP, 0, kD);
  }

  public PIDConstants(double kP) {
    this(kP, 0, 0);
  }
}
