package frc.team4201.lib.command;

import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoggedSubsystem extends SubsystemBase
    implements MechanismLoggerInterface<Distance, Distance, Velocity, Acceleration> {

  public LoggedSubsystem() {}

  @Override
  public Distance getSetpoint() {
    return null;
  }

  @Override
  public Distance getError() {
    return null;
  }

  @Override
  public Distance getPosition() {
    return null;
  }

  @Override
  public Velocity getVelocity() {
    return null;
  }

  @Override
  public Acceleration getAcceleration() {
    return null;
  }
}
