package frc.team4201.lib.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import frc.team4201.lib.command.LoggedSubsystem;

@CustomLoggerFor(SubsystemInterfaceLogger.class)
public class SubsystemInterfaceLogger extends ClassSpecificLogger<SubsystemInterfaceLogger> {

    public SubsystemInterfaceLogger() {
        super(SubsystemInterfaceLogger.class);
    }

    @Override
    public void update(EpilogueBackend backend, SubsystemInterfaceLogger loggedSubsystemInterface) {
        //backend.log(loggedSubsystemInterface.getClass().getTypeParameters());
//        backend.log("Supply Voltage (V)", motor.getSupplyVoltage().getValue());
    }
}
