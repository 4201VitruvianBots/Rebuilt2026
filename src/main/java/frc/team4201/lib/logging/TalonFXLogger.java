package frc.team4201.lib.logging;

import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

import java.util.LinkedHashMap;

import static edu.wpi.first.units.Units.Hertz;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
  private static final LinkedHashMap<TalonFX, StatusSignalCollection> m_signalMap = new LinkedHashMap<>();

  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  public void update(EpilogueBackend backend, TalonFX motor) {
    if(!m_signalMap.containsKey(motor)) {
      System.out.printf("Adding TalonFX %02d to be logged\n", motor.getDeviceID());
      var signals = new StatusSignalCollection();
      signals.addSignals(
          motor.getSupplyVoltage(),
          motor.getSupplyCurrent(),
          motor.getDutyCycle(),
          motor.getMotorVoltage(),
          motor.getStatorCurrent(),
          motor.getPosition(),
          motor.getVelocity(),
          motor.getAcceleration(),
          motor.getClosedLoopReference(),
          motor.getClosedLoopError()
          );
      signals.setUpdateFrequencyForAll(Hertz.of(50));
      motor.optimizeBusUtilization(Hertz.of(1));
      m_signalMap.put(motor, signals);
    }

    backend.log("Supply Voltage (V)", motor.getSupplyVoltage().getValue());
    backend.log("Supply Current (A)", motor.getSupplyCurrent().getValue());

    backend.log("Control Mode", motor.getAppliedControl().getName());

    backend.log("Output (%)", motor.getDutyCycle().getValue());
    backend.log("Output (V)", motor.getMotorVoltage().getValue());
    backend.log("Stator Current (A)", motor.getStatorCurrent().getValue());

    backend.log("Position", motor.getPosition().getValue());
    backend.log("Velocity", motor.getVelocity().getValue());
    backend.log("Acceleration", motor.getAcceleration().getValue());

    backend.log("Setpoint", motor.getClosedLoopReference().getValue());
    backend.log("Error", motor.getClosedLoopError().getValue());

//    System.out.printf("[DEBUG] TalonFX %02d logged data\n", motor.getDeviceID());
  }
}
