package frc.team4201.lib.command;

import edu.wpi.first.units.Measure;

public interface SubsystemLoggerInterface<C extends Measure, P extends Measure, V extends Measure, A extends  Measure> {

    Class<C> getSetpoint();

    Class<P> getPosition();

    Class<P> getVelocity();

    Class<P> getAcceleration();
}
