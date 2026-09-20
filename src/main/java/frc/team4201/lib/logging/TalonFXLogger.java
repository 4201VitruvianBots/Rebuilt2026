package frc.team4201.lib.logging;

import static org.wpilib.units.Units.Hertz;

import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.CAN;
import java.util.LinkedHashMap;
import java.util.Objects;
import org.wpilib.epilogue.CustomLoggerFor;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.logging.ClassSpecificLogger;
import org.wpilib.telemetry.TelemetryTable;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
  private static final LinkedHashMap<TalonFX, StatusSignalCollection> m_signalMap =
      new LinkedHashMap<>();

  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  protected void update(TelemetryTable table, TalonFX motor) {
    if (!m_signalMap.containsKey(motor)) {
      var signals = new StatusSignalCollection();

      signals.addSignals(
          motor.getSupplyVoltage(),
          motor.getSupplyCurrent(),
          motor.getDutyCycle(),
          motor.getMotorVoltage(),
          motor.getStatorCurrent(),
          motor.getPosition());

      if (Epilogue.shouldLog(Logged.Importance.INFO)) {
        signals.addSignals(
            motor.getVelocity(),
            motor.getAcceleration(),
            motor.getClosedLoopReference(),
            motor.getClosedLoopError());
      }
      if (Objects.equals(motor.getNetwork(), CAN.roboRIO)) {
        System.out.printf(
            "Adding TalonFX %02d (%s) to be logged from roboRIO\n",
            motor.getDeviceID(), motor.getDescription());
        signals.setUpdateFrequencyForAll(Hertz.of(50));
      } else {
        System.out.printf(
            "Adding TalonFX %02d (%s) to be logged from canivore\n",
            motor.getDeviceID(), motor.getDescription());
        signals.setUpdateFrequencyForAll(Hertz.of(250));
      }

      motor.optimizeBusUtilization(Hertz.of(1));
      m_signalMap.put(motor, signals);
    }

    table.log("Supply Voltage (V)", motor.getSupplyVoltage().getValue());
    table.log("Supply Current (A)", motor.getSupplyCurrent().getValue());

    table.log("Control Mode", motor.getAppliedControl().getName());

    table.log("Output (%)", motor.getDutyCycle().getValue());
    table.log("Output (V)", motor.getMotorVoltage().getValue());
    table.log("Stator Current (A)", motor.getStatorCurrent().getValue());

    table.log("Position", motor.getPosition().getValue());

    if (Epilogue.shouldLog(Logged.Importance.INFO)) {
      table.log("Velocity", motor.getVelocity().getValue());
      table.log("Acceleration", motor.getAcceleration().getValue());

      table.log("Setpoint", motor.getClosedLoopReference().getValue());
      table.log("Error", motor.getClosedLoopError().getValue());
    }

    //    System.out.printf("[DEBUG] TalonFX %02d logged data\n", motor.getDeviceID());
  }
}
