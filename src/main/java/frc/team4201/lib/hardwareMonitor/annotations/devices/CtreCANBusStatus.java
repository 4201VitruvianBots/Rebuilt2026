package frc.team4201.lib.hardwareMonitor.annotations.devices;

import com.ctre.phoenix6.CANBus;
import frc.team4201.lib.hardwareMonitor.HardwareMonitor.HEALTH_STATUS;
import frc.team4201.lib.hardwareMonitor.annotations.BaseDeviceMonitor;
import frc.team4201.lib.hardwareMonitor.annotations.HealthStatusFor;
import java.util.*;

@HealthStatusFor(CANBus.class)
public class CtreCANBusStatus extends BaseDeviceMonitor<CANBus> {

  public CtreCANBusStatus() {
    super(CANBus.class);
  }

  @Override
  public HEALTH_STATUS checkHealth(CANBus canbus) {
    HEALTH_STATUS canbusHealthStatus = HEALTH_STATUS.HEALTHY;

    var deviceStatus = canbus.getStatus();
    if (deviceStatus.BusOffCount > 0) {
      canbusHealthStatus = HEALTH_STATUS.BAD;
    } else if (deviceStatus.TxFullCount > 0) {
      canbusHealthStatus = HEALTH_STATUS.DEGRADED;
    }

    return canbusHealthStatus;
  }
}
