// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED.LED_STATES;

public class LEDs extends SubsystemBase {
  private DatagramSocket socket;
  private LED_STATES currentState = LED_STATES.DISABLED;
  
  public LEDs() {
    try {
        socket = new DatagramSocket();
    } catch (SocketException e) {
        DriverStation.reportWarning("LEDs: Failed to create DatagramSocket on port 5800 " + e.getMessage(), false);
    }
  }
  
  public void sendBytes(byte[] data) {
    if (socket == null || socket.isClosed()) {
      DriverStation.reportWarning("LEDs: socket is not available for sending.", false);
      return;
    }
    try {
      InetAddress addr = InetAddress.getByName("10.42.1.2");
      DatagramPacket packet = new DatagramPacket(data, data.length, addr, 5800);
      socket.send(packet);
    } catch (Exception e) {
      DriverStation.reportWarning("LEDs: send failed: " + e.getMessage(), false);
    }
  }
  
  public void setState(LED_STATES newState) {
    if (newState != currentState) {
      currentState = newState;
      sendBytes(newState.getAnimation().getBytes(StandardCharsets.UTF_8));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
