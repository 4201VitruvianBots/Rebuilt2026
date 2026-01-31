// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.USB;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

@Logged
public class Controls extends SubsystemBase {
  //   private Vision m_vision;
  private static boolean m_allianceInit;
  private static DriverStation.Alliance m_allianceColor = DriverStation.Alliance.Red;

  private static Alert m_usbAlert =
      new Alert("USB connection alert not properly initialized", AlertType.kError);
  private static Alert m_brownoutAlert =
      new Alert("Brownout alert not properly initialized", AlertType.kWarning);
  private static Alert m_storageAlert =
      new Alert("Storage space alert not properly initialized", AlertType.kWarning);
  private static Alert m_canAlert =
      new Alert("CAN bus alert not properly initialized", AlertType.kError);
  // private static Alert m_visionAlert = new Alert("Vision alert not properly initialized",
  // AlertType.kWarning);

  private static Timer m_brownoutTimer = new Timer();
  private static double m_brownoutLastUpdatedTime = 0.0;

  /** Map of subsystems for Controls to update */
  @NotLogged private final Map<String, Subsystem> m_subsystemMap = new HashMap<>();

  @NotLogged private final Map<String, Alert> alertMap = new HashMap<>();

  /** Creates a new Controls subsystem */
  public Controls() {
    // Alerts for setting up the robot properly
    alertMap.put(
        "allianceInit",
        new Alert("Did not get alliance color from FMS/DS!", Alert.AlertType.kWarning));
    alertMap.put(
        "visionInit", new Alert("Vision Subsystem is not ready!", Alert.AlertType.kWarning));

    // Alerts for when the robot is running
    alertMap.put("radioError", new Alert("Robot Radio not detected!", Alert.AlertType.kError));
    alertMap.put("joystickError", new Alert("Missing joystick detected!", Alert.AlertType.kError));
    alertMap.put("canError", new Alert("CAN bus error detected!", Alert.AlertType.kError));
    initSmartDashboard();
  }

  public static DriverStation.Alliance getAllianceColor() {
    return m_allianceColor;
  }

  public static boolean isRedAlliance() {
    return getAllianceColor() == DriverStation.Alliance.Red;
  }

  public static boolean isBlueAlliance() {
    return getAllianceColor() == DriverStation.Alliance.Blue;
  }

  private void initSmartDashboard() {
    SmartDashboard.putString("Controls/Serial Number", RobotController.getSerialNumber());
  }

  public void registerSubsystem(Subsystem subsystem) {
    if (subsystem != null) {
      m_subsystemMap.put(subsystem.getName(), subsystem);
    } else {
      DriverStation.reportWarning("[Controls] Attempting to register null subsystem!", true);
    }
  }

  public void updateAlerts() {
    // Update USB alerts
    m_usbAlert.set(false);

    // String for USB alert message
    String usbAlertMessage = "The following USB devices are not connected: ";

    if (!DriverStation.isJoystickConnected(USB.driver_xBoxController)) {
      usbAlertMessage += "Driver Xbox Controller, ";
      m_usbAlert.set(true);
    }

    // Remove the last comma and space
    if (usbAlertMessage.endsWith(", ")) {
      usbAlertMessage = usbAlertMessage.substring(0, usbAlertMessage.length() - 2);
    }
    m_usbAlert.setText(usbAlertMessage);

    // Update brownout alert state
    if (RobotController.isBrownedOut()) {
      m_brownoutAlert.setText(
          "A brownout occured less than a minute ago. Please ensure that a fresh battery has been plugged in.");
      m_brownoutAlert.set(true);
      m_brownoutTimer.restart();
      m_brownoutLastUpdatedTime = 0.0;
    }

    // Update brownout alert timing once a minute
    if (m_brownoutTimer.get() - m_brownoutLastUpdatedTime > 60.0) {
      int minutesCount = (int) Math.round(m_brownoutTimer.get() / 60.0);
      m_brownoutAlert.setText(
          "A brownout occured "
              + minutesCount
              + " minute"
              + (minutesCount == 1 ? "" : "s")
              + " ago. Please ensure that a fresh battery has been plugged in."); // this condition
      // being flipped
      // makes NO SENSE
      // but whatever
      m_brownoutLastUpdatedTime = m_brownoutTimer.get();
    }

    // Get the amount of storage space left
    File root = new File("/");
    long freeSpaceMB = root.getFreeSpace() / 1048576;
    if (!m_storageAlert.get() && freeSpaceMB < 512.0) {
      m_storageAlert.setText(
          "There is only "
              + freeSpaceMB
              + " MB of storage space left on the RoboRIO. Consider deleting old logs to free up space.");
      m_storageAlert.set(true);
    }

    // Update CAN alerts
    m_canAlert.set(false);

    // String for CAN alert message
    String canAlertMessage = "The following CAN devices are not connected: ";

    // Remove the last comma and space
    if (canAlertMessage.endsWith(", ")) {
      canAlertMessage = canAlertMessage.substring(0, usbAlertMessage.length() - 2);
    }
    m_canAlert.setText(canAlertMessage);

    // // Update vision alerts
    // m_visionAlert.set(false);

    // if (m_subsystemMap.containsKey("Vision")) {
    //     var visionSubsystem = (Vision) m_subsystemMap.get("Vision");
    //     if (!visionSubsystem.isLimelightFConnected() && !visionSubsystem.isLimelightBConnected())
    // {
    //         m_visionAlert.setText("Both Limelights are disconnected");
    //         m_visionAlert.set(true);
    //     } else if (!visionSubsystem.isLimelightFConnected()) {
    //         m_visionAlert.setText("The front Limelight is disconnected");
    //         m_visionAlert.set(true);
    //     } else if (!visionSubsystem.isLimelightBConnected()) {
    //         m_visionAlert.setText("The back Limelight is disconnected");
    //         m_visionAlert.set(true);
    //     }
    // }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              a -> {
                m_allianceColor = a;
                m_allianceInit = true;
              });
      alertMap.get("allianceInit").set(!m_allianceInit);
    }
  }
}
