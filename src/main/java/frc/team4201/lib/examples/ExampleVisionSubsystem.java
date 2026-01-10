package frc.team4201.lib.examples;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import frc.team4201.lib.command.LoggedSubsystem;
import frc.team4201.lib.command.SwerveSubsystem;
import frc.team4201.lib.vision.Limelight;

public class ExampleVisionSubsystem extends LoggedSubsystem {
  SwerveSubsystem m_swerve;
  Limelight[] m_limelights = new Limelight[2];

  public ExampleVisionSubsystem() {
    m_limelights[0] = new Limelight("LimelightL", "10.42.1.11");
    m_limelights[1] = new Limelight("LimelightR", "10.42.1.12");
  }

  public void registerSwerveSubsystem(SwerveSubsystem swerve) {
    m_swerve = swerve;
  }

  @Override
  public void periodic() {
    super.periodic();

    for (var limelight : m_limelights) {
      if (limelight.process()) {
        if (m_swerve != null) {
          if (m_swerve.getYawRate().gt(DegreesPerSecond.of(720))) {
            continue;
          }

          limelight
              .getLastValidMeasurement()
              .ifPresent((meas) -> m_swerve.addVisionMeasurement(meas));
        }
      }
    }
  }
}
