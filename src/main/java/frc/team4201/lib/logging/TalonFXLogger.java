package frc.team4201.lib.logging;

import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.epilogue.CustomLoggerFor;
import org.wpilib.epilogue.logging.ClassSpecificLogger;
import org.wpilib.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {

  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  public void update(EpilogueBackend backend, TalonFX motor) {
    backend.log("Supply Voltage (V)", motor.getSupplyVoltage().refresh().getValue());
    backend.log("Supply Current (A)", motor.getSupplyCurrent().refresh().getValue());

    backend.log("Control Mode", motor.getAppliedControl().getName());

    backend.log("Output (%)", motor.getThrottle());
    backend.log("Output (V)", motor.getMotorVoltage().refresh().getValue());
    backend.log("Stator Current (A)", motor.getStatorCurrent().refresh().getValue());

    backend.log("Position", motor.getPosition().refresh().getValue());
    backend.log("Velocity", motor.getVelocity().refresh().getValue());
    backend.log("Acceleration", motor.getAcceleration().refresh().getValue());

    backend.log("Setpoint", motor.getClosedLoopReference().refresh().getValue());
    backend.log("Error", motor.getClosedLoopError().refresh().getValue());
  }
}
