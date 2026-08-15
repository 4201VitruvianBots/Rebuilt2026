// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.wpilib.driverstation.*;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.NotLogged;
import org.wpilib.math.filter.MedianFilter;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.driverstation.Alert.Level;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.SubsystemBase;
import frc.robot.constants.FIELD;
import frc.robot.constants.ROBOT.USB;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

@Logged
public class Controls extends SubsystemBase {
  private static boolean m_allianceInit;
  private static Alliance m_allianceColor = Alliance.RED;

  // private static Alert m_visionAlert = new Alert("Vision alert not properly initialized",
  // AlertType.kWarning);

  private static Timer m_brownoutTimer = new Timer();
  private static double m_brownoutLastUpdatedTime = 0.0;

  /** Map of subsystems for Controls to update */
  @NotLogged private final Map<String, Subsystem> m_subsystemMap = new HashMap<>();

  @NotLogged
  private final Map<String, Alert> alertMap =
      Map.ofEntries(
          Map.entry(
              "usb", new Alert("USB connection alert not properly initialized", Level.HIGH)),
          Map.entry(
              "brownout", new Alert("Brownout alert not properly initialized", Level.MEDIUM)),
          Map.entry(
              "storage",
              new Alert("Storage space alert not properly initialized", Level.MEDIUM)),
          Map.entry(
              "epilogue",
              new Alert("Epilogue Runtime average is > 0.04 seconds!", Level.MEDIUM)),
          // Alerts for setting up the robot properly
          Map.entry(
              "allianceInit",
              new Alert("Did not get alliance color from FMS/DS!", Level.MEDIUM)),
          Map.entry(
              "vision", new Alert("Vision Subsystem is not ready!", Level.MEDIUM)),

          // Alerts for when the robot is running
          Map.entry("radioError", new Alert("Robot Radio not detected!", Level.HIGH)),
          Map.entry(
              "joystickError", new Alert("Missing joystick detected!", Level.HIGH)),
          Map.entry("canError", new Alert("CAN bus error detected!", Level.HIGH)));

  private final MedianFilter epilogueBuffer = new MedianFilter(20);
  private final DoubleSubscriber epilogueRuntimeSub =
      NetworkTableInstance.getDefault().getDoubleTopic("Epilogue/Stats/Last Run").subscribe(0.0);

  /** Creates a new Controls subsystem */
  public Controls() {
    initSmartDashboard();
  }

  public void registerSubsystem(Subsystem subsystem) {
    if (subsystem != null) {
      m_subsystemMap.put(subsystem.getName(), subsystem);
    } else {
      DriverStationErrors.reportWarning("[Controls] Attempting to register null subsystem!", true);
    }
  }

  public static Alliance getAllianceColor() {
    return m_allianceColor;
  }

  public static boolean isRedAlliance() {
    return getAllianceColor() == Alliance.RED;
  }

  public static boolean isBlueAlliance() {
    return getAllianceColor() == Alliance.BLUE;
  }

  private void initSmartDashboard() {
    SmartDashboard.putString("Controls/Serial Number", RobotController.getSerialNumber());
  }

  public void updateAlerts() {
    // Update USB alerts
    alertMap.get("usb").set(false);

    // String for USB alert message
    String usbAlertMessage = "The following USB devices are not connected: ";

    // TODO: Change this to a loop/array for String.join()
    if (!DriverStationBackend.isJoystickConnected(USB.driver_xBoxController)) {
      usbAlertMessage += String.join(", ", "Driver Xbox Controller");
      alertMap.get("usb").setText(usbAlertMessage);
    }

    alertMap.get("usb").set(true);

    // Update brownout alert state
    // TODO: WPILib6 alpha causes a HAL error with RobotController.isBrownedOut()
    // if (RobotController.isBrownedOut()) {
    //   alertMap
    //       .get("brownout")
    //       .setText(
    //           "A brownout occurred less than a minute ago. Please ensure that a fresh battery has been plugged in.");
    //   alertMap.get("brownout").set(true);
    //   m_brownoutTimer.restart();
    //   m_brownoutLastUpdatedTime = 0.0;
    // }

    // Update brownout alert timing once a minute
    if (m_brownoutTimer.get() - m_brownoutLastUpdatedTime > 60.0) {
      int minutesCount = (int) Math.round(m_brownoutTimer.get() / 60.0);
      alertMap
          .get("brownout")
          .setText(
              "A brownout occurred "
                  + minutesCount
                  + " minute"
                  + (minutesCount == 1 ? "" : "s")
                  + " ago. Please ensure that a fresh battery has been plugged in."); // this
      // condition
      // being flipped
      // makes NO SENSE
      // but whatever
      m_brownoutLastUpdatedTime = m_brownoutTimer.get();
    }

    // Get the amount of storage space left
    File root = new File("/");
    long freeSpaceMB = root.getFreeSpace() / 1048576;
    if (!alertMap.get("storage").get() && freeSpaceMB < 512.0) {
      alertMap
          .get("storage")
          .setText(
              "There is only "
                  + freeSpaceMB
                  + " MB of storage space left on the RoboRIO. Consider deleting old logs to free up space.");
      alertMap.get("storage").set(true);
    }

    // Update CAN alerts
    alertMap.get("canError").set(false);

    // String for CAN alert message
    String canAlertMessage = "The following CAN devices are not connected: ";
    //    canAlertMessage += String.join(", ")
    //    alertMap.get("canError").setText(canAlertMessage);

    // Update vision alerts
    alertMap.get("vision").set(false);
    m_subsystemMap.computeIfPresent(
        "Vision",
        (v, s) -> {
          var vision = (Vision) s;
          if (!vision.lllConnected() && !vision.llrConnected()) {
            alertMap.get("vision").setText("Both Limelights are disconnected");
            alertMap.get("vision").set(true);
          } else if (!vision.lllConnected()) {
            alertMap.get("vision").setText("The left Limelight is disconnected");
            alertMap.get("vision").set(true);
          } else if (!vision.llrConnected()) {
            alertMap.get("vision").setText("The right Limelight is disconnected");
            alertMap.get("vision").set(true);
          }
          return vision;
        });

    var avgRuntime = epilogueBuffer.calculate(epilogueRuntimeSub.get());
    alertMap.get("epilogue").set(avgRuntime > 40);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAlerts();

    if (RobotState.isDisabled()) {
      DriverStationBackend.getAlliance()
          .ifPresent(
              a -> {
                m_allianceColor = a;
                m_allianceInit = true;
              });
      alertMap.get("allianceInit").set(!m_allianceInit);

      // Update field constants
      FIELD.updateConstants();
    }
  }
}
