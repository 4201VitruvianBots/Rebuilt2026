package frc.team4201.lib.logging;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(CANBus.class)
public class CANBusLogger extends ClassSpecificLogger<CANBus> {

  public CANBusLogger() {
    super(CANBus.class);
  }

  @Override
  public void update(EpilogueBackend backend, CANBus bus) {
    var status = bus.getStatus();
    backend.log("Status Code", status.Status.getName());
    backend.log("Utilization", status.BusUtilization);
    backend.log("Off Count", status.BusOffCount);
    backend.log("TxFull Count", status.TxFullCount);
    backend.log("RxError Count", status.REC);
    backend.log("TxError Count", status.TEC);
  }
}
