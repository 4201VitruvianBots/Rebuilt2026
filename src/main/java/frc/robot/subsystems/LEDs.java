// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWM;
import frc.robot.Constants.LED.LED_STATES;

public class LEDs extends SubsystemBase {
  private LED_STATES currentState = LED_STATES.DISABLED;
  
  // We'll be using a WS2812 LED strip controlled through PWM
  private AddressableLED m_led = new AddressableLED(PWM.kLED);

  public LEDs() {
    m_led.start();
  }
  
  public void setState(LED_STATES state) {
    if (state != currentState) {
      currentState = state;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
