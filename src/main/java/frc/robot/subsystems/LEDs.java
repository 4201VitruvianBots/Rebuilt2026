// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED.LED_STATES;

public class LEDs extends SubsystemBase {
  private SerialPort ledPort = new SerialPort(9600, SerialPort.Port.kUSB1);
  private LED_STATES currentState = LED_STATES.DISABLED;

  public LEDs() {}

  public void setState(LED_STATES newState) {
    if (newState != currentState) {
      currentState = newState;
      System.out.println("LEDs: Changing state to " + newState.getAnimation());
      ledPort.writeString(newState.getAnimation() + "\n");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
