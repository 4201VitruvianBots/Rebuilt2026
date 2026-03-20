package frc.team4201.lib.hardwareMonitor.annotations.devices;

import static com.ctre.phoenix6.StatusCode.*;
import static com.ctre.phoenix6.signals.MotorOutputStatusValue.*;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import edu.wpi.first.wpilibj.RobotBase;
import frc.team4201.lib.hardwareMonitor.HardwareMonitor.HEALTH_STATUS;
import frc.team4201.lib.hardwareMonitor.annotations.BaseDeviceMonitor;
import frc.team4201.lib.hardwareMonitor.annotations.HealthStatusFor;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredDevice;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

@HealthStatusFor(TalonFX.class)
public class TalonFXStatus extends BaseDeviceMonitor<TalonFX> {
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
  static Map<HEALTH_STATUS, EnumSet<MotorOutputStatusValue>> MOTOR_OUTPUT_BITMASK =
      new LinkedHashMap<>(
          Map.ofEntries(
              Map.entry(HEALTH_STATUS.BAD, EnumSet.of(Unknown, Off)),
              Map.entry(HEALTH_STATUS.DEGRADED, EnumSet.of(Motoring, DiscordantMotoring))));

  public TalonFXStatus() {
    super(TalonFX.class);
  }

  public LinkedList<BaseStatusSignal> getSignals(
      TalonFX motor, MonitoredDevice.DEVICE_TYPE deviceType) {
    var signals =
        new LinkedList<BaseStatusSignal>(
            List.of(
                motor.getSupplyVoltage(),
                motor.getSupplyCurrent(),
                motor.getDeviceTemp(),
                motor.getMotorVoltage(),
                motor.getStatorCurrent()));

    if (deviceType == MonitoredDevice.DEVICE_TYPE.PRIMARY) {
      var primarySignals =
          new LinkedList<BaseStatusSignal>(
              List.of(
                  motor.getPosition(),
                  motor.getVelocity(),
                  motor.getAcceleration(),
                  motor.getClosedLoopReference(),
                  motor.getClosedLoopError()));

      signals.addAll(primarySignals);
    }

    if (RobotBase.isSimulation()) {
      BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(250), signals);
    } else if (deviceType == MonitoredDevice.DEVICE_TYPE.PRIMARY) {
      BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(100), signals);
    } else {
      BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), signals);
    }

    if (!RobotBase.isSimulation()) motor.optimizeBusUtilization();

    return signals;
  }

  @Override
  public HEALTH_STATUS checkHealth(TalonFX motor) {
    AtomicReference<HEALTH_STATUS> motorHealthStatus = new AtomicReference<>(HEALTH_STATUS.HEALTHY);

    var motorOutputStatusSignal = motor.getMotorOutputStatus();
    if (motorOutputStatusSignal.getStatus() != OK) {
      STATUS_CODE_BITMASKS.forEach(
          (s, v) -> {
            if (v.contains(motorOutputStatusSignal.getStatus())) {
              motorHealthStatus.set(s);
            }
          });
    } else {
      MOTOR_OUTPUT_BITMASK.forEach(
          (s, v) -> {
            if (v.contains(motorOutputStatusSignal.getValue())) {
              motorHealthStatus.set(s);
            }
          });
    }

    // TODO: To implement
    var faultStatus = motor.getFaultField();

    var stickFaultStatus = motor.getStickyFaultField();

    return motorHealthStatus.get();
  }
}
