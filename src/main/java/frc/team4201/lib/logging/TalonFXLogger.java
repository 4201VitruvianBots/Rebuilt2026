package frc.team4201.lib.logging;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {

  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  public void update(EpilogueBackend backend, TalonFX motor) {
    backend.log("Supply Voltage (V)", motor.getSupplyVoltage().getValue());
    backend.log("Supply Current (A)", motor.getSupplyCurrent().getValue());

    backend.log("Control Mode", motor.getAppliedControl().getName());

    backend.log("Output (%)", motor.get());
    backend.log("Output (V)", motor.getMotorVoltage().getValue());
    backend.log("Stator Current (A)", motor.getStatorCurrent().getValue());

    backend.log("Velocity", motor.getVelocity().getValue());
    backend.log("Acceleration", motor.getAcceleration().getValue());

    backend.log("Setpoint", motor.getClosedLoopReference().getValue());
    backend.log("Error", motor.getClosedLoopError().getValue());
  }
}
