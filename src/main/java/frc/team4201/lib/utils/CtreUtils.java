package frc.team4201.lib.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredDevice;

import java.util.HashMap;
import java.util.LinkedHashMap;

/** Utility class to interact with CTRE libraries. */
public final class CtreUtils {
  private static LinkedHashMap<String, StatusSignalCollection> allPrimarySignals = new LinkedHashMap<>();
  private static LinkedHashMap<String, StatusSignalCollection> allSecondarySignals = new LinkedHashMap<>();
  /**
   * Initialize Phoenix Server by creating a dummy device. We do this so that the CANCoders don't
   * get configured before Phoenix Server is up, which causes issues with encoder offsets not being
   * set/applied properly.
   */
  @Deprecated
  public static void initPhoenixServer() {
    var alert =
        new Alert("Starting Phoenix Server at: " + Timer.getFPGATimestamp(), AlertType.kInfo);
    alert.set(true);
    if (RobotBase.isReal()) {
      TalonFX dummy = new TalonFX(0, new CANBus());
      Timer.delay(5);
      dummy.close();
    }
    alert.setText("Phoenix Server finished Init at: " + Timer.getFPGATimestamp());
  }

  /**
   * Apply a CTRE Configuration to a CTRE Device. Will retry until it gets StatusCode.OK or until it
   * gives up after 5 tries. If it gives up, an {@link Alert} will be sent to warn the user that it
   * has failed.
   *
   * @param device CTRE Device to configure
   * @param config CTRE Configuration to apply
   * @return boolean true if successful
   */
  public static boolean configureDevice(ParentDevice device, ParentConfiguration config) {
    StatusCode deviceStatus = StatusCode.StatusCodeNotInitialized;
    int counter = 0;
    int counterLimit = RobotBase.isReal() ? 5 : 1;
    while (counter++ < counterLimit) {
      try {
        if (device instanceof TalonFX) {
          deviceStatus = ((TalonFX) device).getConfigurator().apply((TalonFXConfiguration) config);
        } else if (device instanceof CANcoder) {
          deviceStatus =
              ((CANcoder) device).getConfigurator().apply((CANcoderConfiguration) config);
        } else {
          throw new IllegalArgumentException(
              String.format("CtreUtils [ERROR] Unsupported Class type %s!\n", device.getClass()));
        }
      } catch (Exception e) {
        System.out.printf(
            "CtreUtils [ERROR] Could not apply config to CTRE device (%020d)\n",
            device.getDeviceID());
        throw e;
      }
      if (deviceStatus == StatusCode.OK) {
        System.out.printf(
            "CTRE Device ID: %02d - Successfully configured!\n", device.getDeviceID());
        return deviceStatus.isOK();
      }
      if (RobotBase.isReal()) Timer.delay(0.02);
    }
    var alert =
        new Alert(
            String.format(
                "Could not apply configs to TalonFx ID: %d. Error code: %s",
                device.getDeviceID(), deviceStatus),
            AlertType.kError);
    alert.set(true);
    return false;
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

  public static void optimizeSignals() {
    ParentDevice.optimizeBusUtilizationForAll();
  }

  public static void waitForPrimarySignals() {
    allPrimarySignals.forEach((canbus, signals) ->
    {signals.waitForAll(0.002);});
  }
}
