// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWM;
import frc.robot.Constants.LED;
import frc.robot.Constants.LED.LED_STATES;

public class LEDs extends SubsystemBase {
  private LED_STATES currentState = LED_STATES.DISABLED;
  
  // We'll be using a WS2812 LED strip controlled through PWM
  private AddressableLED m_led = new AddressableLED(PWM.kLED);
  
  private AddressableLEDBuffer m_ledBuffer;
  private AddressableLEDSim m_ledSim = new AddressableLEDSim(m_led);

  public LEDs() {
    m_ledBuffer = new AddressableLEDBuffer(LED.numLEDs);
    m_led.setLength(m_ledBuffer.getLength());
    
    m_led.setData(m_ledBuffer);
    m_led.start();
  }
  
  public void setState(LED_STATES state) {
    if (state != currentState) {
      currentState = state;
      LEDPattern base;
      LEDPattern pattern;
      switch (currentState) {
        case DISABLED:
            base = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlack));
            pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
            pattern.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
            break;
        case IDLE:
            base = LEDPattern.steps(Map.of(0, Color.kGreen, 0.5, Color.kBlack));
            pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
            pattern.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
            break;
        case DRIVING:
            break;
        case INTAKING:
            break;
        case SHOOTING:
            break;
        case CLIMBING:
            break;
      }
    }
  }
  
  public String getDataString() {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      Color c = m_ledBuffer.getLED(i);
      int r = (int) Math.round(Math.max(0, Math.min(1, c.red)) * 255);
      int g = (int) Math.round(Math.max(0, Math.min(1, c.green)) * 255);
      int b = (int) Math.round(Math.max(0, Math.min(1, c.blue)) * 255);
      sb.append(String.format("LED %d: R=%d G=%d B=%d\n", i, r, g, b));
    }
    return sb.toString();
  }
  
  public LED_STATES getState() {
    return currentState;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
