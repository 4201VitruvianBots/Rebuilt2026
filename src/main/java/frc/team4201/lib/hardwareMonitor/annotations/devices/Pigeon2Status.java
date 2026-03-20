package frc.team4201.lib.hardwareMonitor.annotations.devices;

import static com.ctre.phoenix6.StatusCode.*;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.RobotBase;
import frc.team4201.lib.hardwareMonitor.HardwareMonitor.HEALTH_STATUS;
import frc.team4201.lib.hardwareMonitor.annotations.BaseDeviceMonitor;
import frc.team4201.lib.hardwareMonitor.annotations.HealthStatusFor;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredDevice;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

@HealthStatusFor(Pigeon2.class)
public class Pigeon2Status extends BaseDeviceMonitor<Pigeon2> {
  static Map<HEALTH_STATUS, EnumSet<StatusCode>> STATUS_CODE_BITMASKS =
      new LinkedHashMap<>(
          Map.ofEntries(
              Map.entry(
                  HEALTH_STATUS.BAD,
                  EnumSet.of(
                      GeneralError,
                      BufferFailure,
                      NoDevicesOnBus,
                      AppTooOld,
                      DeviceDidNotRespondToDiagReq,
                      DeviceIsNull,
                      UsingProFeatureOnUnlicensedDevice)),
              Map.entry(
                  HEALTH_STATUS.DEGRADED,
                  EnumSet.of(GeneralWarning, CanMessageStale, RxTimeout, TxTimeout))));
  static EnumSet<StatusCode> STATUS_CODE_IGNORE_BITMASK = EnumSet.of(OK);

  public Pigeon2Status() {
    super(Pigeon2.class);
  }

  public LinkedList<BaseStatusSignal> getSignals(
      Pigeon2 pigeon2, MonitoredDevice.DEVICE_TYPE deviceType) {
    var signals = new LinkedList<BaseStatusSignal>(List.of(pigeon2.getYaw()));

    if (deviceType == MonitoredDevice.DEVICE_TYPE.PRIMARY) {
      //      var primarySignals =
      //          new LinkedList<BaseStatusSignal>(
      //              List.of(pigeon2.getAngularVelocityZDevice(), pigeon2.getAccelerationZ()));
      //
      //      signals.addAll(primarySignals);
    }

    if (RobotBase.isSimulation()) {
      BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(250), signals);
    } else if (deviceType == MonitoredDevice.DEVICE_TYPE.PRIMARY) {
      BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(100), signals);
    } else {
      BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), signals);
    }

    if (!RobotBase.isSimulation()) pigeon2.optimizeBusUtilization();

    return signals;
  }

  @Override
  public HEALTH_STATUS checkHealth(Pigeon2 pigeon2) {
    AtomicReference<HEALTH_STATUS> motorHealthStatus = new AtomicReference<>(HEALTH_STATUS.HEALTHY);

    var deviceStatus = pigeon2.getYaw();
    if (deviceStatus.getStatus() != OK) {
      STATUS_CODE_BITMASKS.forEach(
          (s, v) -> {
            if (v.contains(deviceStatus.getStatus())) {
              motorHealthStatus.set(s);
            }
          });
    }

    // TODO: To implement
    var faultStatus = pigeon2.getFaultField();

    var stickFaultStatus = pigeon2.getStickyFaultField();

    return motorHealthStatus.get();
  }
}
