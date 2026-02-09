package frc.team4201.lib.logging;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4201.lib.command.LoggedSubsystem;

@CustomLoggerFor(LoggedSubsystem.class)
public class SubsystemLogger extends ClassSpecificLogger<LoggedSubsystem> {

    public SubsystemLogger() {
        super(LoggedSubsystem.class);
    }

    @Override
    public void update(EpilogueBackend backend, LoggedSubsystem subsystem) {
//        backend.log("Supply Voltage (V)", motor.getSupplyVoltage().getValue());
    }
}
