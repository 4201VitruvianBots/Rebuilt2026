package frc.team4201.lib.hardwareMonitor;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredDevice;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredSubsystem;
import frc.team4201.lib.hardwareMonitor.annotations.devices.CANcoderStatus;
import frc.team4201.lib.hardwareMonitor.annotations.devices.CtreCANBusStatus;
import frc.team4201.lib.hardwareMonitor.annotations.devices.Pigeon2Status;
import frc.team4201.lib.hardwareMonitor.annotations.devices.TalonFXStatus;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.*;

public class HardwareMonitor {
  public enum HEALTH_STATUS {
    BAD,
    DEGRADED,
    INVALID_CONFIG,
    UNKNOWN,
    HEALTHY
  }

  static Map<String, HEALTH_STATUS> subsystemHealth = new LinkedHashMap<>();
  static Map<String, Map<String, HEALTH_STATUS>> subsystemDeviceHealth = new LinkedHashMap<>();

  static LinkedList<CANBus> canBuses = new LinkedList<>();
  static Map<String, LinkedList<Device>> primarySubsystemDevices = new LinkedHashMap<>();
  static Map<String, LinkedList<Device>> secondarySubsystemDevices = new LinkedHashMap<>();
  static Frequency primaryUpdateRate = Hertz.of(100);
  static Frequency secondaryUpdateRate = Hertz.of(50);
  static Map<String, Runnable> subsystemInit = new LinkedHashMap<>();

  static TalonFXStatus talonFXStatus = new TalonFXStatus();
  static CANcoderStatus cancoderStatus = new CANcoderStatus();
  static Pigeon2Status pigeon2Status = new Pigeon2Status();
  static CtreCANBusStatus canBusStatus = new CtreCANBusStatus();

  // TODO: Clean this up in the
  public static LinkedHashMap<String, StatusSignalCollection> allPrimarySignals =
      new LinkedHashMap<>();
  public static LinkedHashMap<String, StatusSignalCollection> allSecondarySignals =
      new LinkedHashMap<>();

  public static void processRobotContainer(RobotContainer robotContainer)
      throws IllegalAccessException {
    Class<? extends RobotContainer> clazz = robotContainer.getClass();

    // Iterate over all declared fields in the class
    for (Field field : clazz.getDeclaredFields()) {
      // Check if the field is a Subsystem and implements MonitoredSubsystem
      if (Subsystem.class.isAssignableFrom(field.getType())) {
        if (!MonitoredSubsystem.class.isAssignableFrom(field.getType())) {
          System.out.printf(
              "[WARN] Subsystem '%s' does not implement MonitoredSubsystem!\n", field.getName());
        }

        field.setAccessible(true);

        var subsystemInstance = (Subsystem) field.get(robotContainer);

        // Only add the subsystem if it has been initialized
        if (subsystemInstance != null) {
          addSubsystemDevices(subsystemInstance);
        }
      }
    }

    //    System.out.printf("Subsystem %s Has device %s\n", subsystem.getName(),
    // primaryDevices.get(0).monitored.name());
  }

  private static void addSubsystemDevices(Subsystem subsystem) throws IllegalAccessException {
    LinkedList<Device> primaryDevices = new LinkedList<>();
    LinkedList<Device> secondaryDevices = new LinkedList<>();

    // Iterate over all declared fields in the current class level
    for (Field field : subsystem.getClass().getDeclaredFields()) {
      // Check if the field has the @Monitored annotation
      if (field.isAnnotationPresent(MonitoredDevice.class)) {
        // Make the field accessible if it's private
        field.setAccessible(true);

        var deviceInstance = field.get(subsystem);

        // Only add the device if it has been initialized
        if (deviceInstance != null) {
          var deviceAnnotation = field.getAnnotation(MonitoredDevice.class);

          var device = getDevice(deviceAnnotation, deviceInstance);

          if (device.annotation.type().equals(MonitoredDevice.DEVICE_TYPE.PRIMARY)) {
            primaryDevices.add(device);
          } else {
            secondaryDevices.add(device);
          }
        }
      }
    }

    if (subsystem instanceof MonitoredSubsystem) {
      try {
        Method method = subsystem.getClass().getMethod("initDevices");

        // Only add to the init map if the method is overridden and not the default implementation
        if (!method.getDeclaringClass().equals(MonitoredSubsystem.class)) {
          Runnable deviceInit =
              () -> {
                try {
                  method.invoke(subsystem);
                } catch (IllegalAccessException | InvocationTargetException e) {
                  throw new RuntimeException(e);
                }
              };
          subsystemInit.put(subsystem.getName(), deviceInit);
        }
      } catch (NoSuchMethodException e) {
        // This shouldn't happen if it implements MonitoredSubsystem as it has a default impl
        System.out.printf(
            "[WARN] Could not find initDevices method for subsystem '%s'\n", subsystem.getName());
      }
    }

    if (!primaryDevices.isEmpty()) {
      primarySubsystemDevices.put(subsystem.getName(), primaryDevices);
      subsystemHealth.put(subsystem.getName(), HEALTH_STATUS.HEALTHY);
    }
    if (!secondaryDevices.isEmpty()) {
      secondarySubsystemDevices.put(subsystem.getName(), secondaryDevices);
    }

    if (RobotBase.isSimulation()) {
      primaryUpdateRate = Hertz.of(250);
      secondaryUpdateRate = Hertz.of(250);
    }

    allPrimarySignals.forEach(
        (canbus, signals) -> signals.setUpdateFrequencyForAll(primaryUpdateRate));
    allSecondarySignals.forEach(
        (canbus, signals) -> signals.setUpdateFrequencyForAll(secondaryUpdateRate));
  }

  public record Device(MonitoredDevice annotation, Object device) {}

  private static Device getDevice(MonitoredDevice deviceAnnotation, Object deviceInstance) {
    //    if (!deviceAnnotation.skipDefaultInit()) {
    if (deviceInstance instanceof TalonFX) {
      var signals = talonFXStatus.getSignals((TalonFX) deviceInstance, deviceAnnotation.type());
      addCtreSignals(
          deviceAnnotation.canbus(),
          deviceAnnotation.type(),
          signals.toArray(new BaseStatusSignal[0]));

    } else if (deviceInstance instanceof CANcoder) {
      var signals = cancoderStatus.getSignals((CANcoder) deviceInstance, deviceAnnotation.type());
      addCtreSignals(
          deviceAnnotation.canbus(),
          deviceAnnotation.type(),
          signals.toArray(new BaseStatusSignal[0]));
    } else if (deviceInstance instanceof Pigeon2) {
      var signals = pigeon2Status.getSignals((Pigeon2) deviceInstance, deviceAnnotation.type());
      addCtreSignals(
          deviceAnnotation.canbus(),
          deviceAnnotation.type(),
          signals.toArray(new BaseStatusSignal[0]));
    } else if (deviceInstance instanceof CANBus) {
      System.out.printf("Registering CANBus: %s\n", deviceAnnotation.name());
      canBuses.add((CANBus) deviceInstance);
    } else {
      throw new IllegalArgumentException(
          String.format("Unsupported device type %s!", deviceInstance.getClass()));
    }
    //    }
    return new Device(deviceAnnotation, deviceInstance);
  }

  public static void addCtreSignals(
      String canbus, MonitoredDevice.DEVICE_TYPE type, BaseStatusSignal... signals) {
    if (type == MonitoredDevice.DEVICE_TYPE.PRIMARY) {
      allPrimarySignals.putIfAbsent(canbus, new StatusSignalCollection());
      allPrimarySignals.get(canbus).addSignals(signals);
    } else {
      allSecondarySignals.putIfAbsent(canbus, new StatusSignalCollection());
      allSecondarySignals.get(canbus).addSignals(signals);
    }
  }

  public static void initAllSubsystems() {
    subsystemInit.forEach((key, value) -> initSubsystem(key));
  }

  public static void initSubsystem(String subsystemName) {
    System.out.printf("Running %s:initDevices()...\n", subsystemName);
    subsystemInit.get(subsystemName).run();
  }

  public static void checkRobotHealth() {
    canBuses.forEach(canbus -> canBusStatus.checkHealth(canbus));

    primarySubsystemDevices.forEach(
        (subsystemName, devices) -> {
          // Reset subsystem health to HEALTHY before checking devices
          subsystemHealth.put(subsystemName, HEALTH_STATUS.HEALTHY);

          devices.forEach(
              obj -> {
                if (obj.device instanceof TalonFX) {
                  var deviceHealth = talonFXStatus.checkHealth((TalonFX) obj.device);

                  if (deviceHealth.ordinal() < subsystemHealth.get(subsystemName).ordinal()) {
                    subsystemHealth.replace(subsystemName, deviceHealth);
                  }
                } else if (obj.device instanceof CANcoder) {
                  var deviceHealth = cancoderStatus.checkHealth((CANcoder) obj.device);

                  if (deviceHealth.ordinal() < subsystemHealth.get(subsystemName).ordinal()) {
                    subsystemHealth.replace(subsystemName, deviceHealth);
                  }
                } else if (obj.device instanceof Pigeon2) {
                  var deviceHealth = pigeon2Status.checkHealth((Pigeon2) obj.device);

                  if (deviceHealth.ordinal() < subsystemHealth.get(subsystemName).ordinal()) {
                    subsystemHealth.replace(subsystemName, deviceHealth);
                  }
                } else if (obj.device instanceof CANBus) {
                  var deviceHealth = canBusStatus.checkHealth((CANBus) obj.device);

                  if (deviceHealth.ordinal() < subsystemHealth.get(subsystemName).ordinal()) {
                    subsystemHealth.replace(subsystemName, deviceHealth);
                  }
                }
              });
        });

    secondarySubsystemDevices.forEach(
        (subsystemName, devices) -> {
          // Reset subsystem health to HEALTHY before checking devices
          subsystemHealth.put(subsystemName, HEALTH_STATUS.HEALTHY);

          devices.forEach(
              obj -> {
                if (obj.device instanceof TalonFX) {
                  var deviceHealth = talonFXStatus.checkHealth((TalonFX) obj.device);

                  if (deviceHealth.ordinal() < subsystemHealth.get(subsystemName).ordinal()) {
                    subsystemHealth.replace(subsystemName, deviceHealth);
                  }
                } else if (obj.device instanceof CANcoder) {
                  var deviceHealth = cancoderStatus.checkHealth((CANcoder) obj.device);

                  if (deviceHealth.ordinal() < subsystemHealth.get(subsystemName).ordinal()) {
                    subsystemHealth.replace(subsystemName, deviceHealth);
                  }
                } else if (obj.device instanceof Pigeon2) {
                  var deviceHealth = pigeon2Status.checkHealth((Pigeon2) obj.device);

                  if (deviceHealth.ordinal() < subsystemHealth.get(subsystemName).ordinal()) {
                    subsystemHealth.replace(subsystemName, deviceHealth);
                  }
                } else if (obj.device instanceof CANBus) {
                  var deviceHealth = canBusStatus.checkHealth((CANBus) obj.device);

                  if (deviceHealth.ordinal() < subsystemHealth.get(subsystemName).ordinal()) {
                    subsystemHealth.replace(subsystemName, deviceHealth);
                  }
                }
              });
        });
  }
}
