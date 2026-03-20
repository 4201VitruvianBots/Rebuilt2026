package frc.team4201.lib.command;

import edu.wpi.first.units.Measure;

public interface MechanismLoggerInterface<
    TSetpoint extends Measure,
    TPosition extends Measure,
    TVelocity extends Measure,
    TAcceleration extends Measure> {

  TSetpoint getSetpoint();

  TSetpoint getError();

  TPosition getPosition();

  TVelocity getVelocity();

  TAcceleration getAcceleration();
}
