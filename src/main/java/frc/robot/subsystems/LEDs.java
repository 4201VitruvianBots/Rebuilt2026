// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LED;

public class LEDs extends SubsystemBase {
  private AddressableLED m_led = new AddressableLED(LED.kPWMPort);
  
  private LEDPattern rainbow;
  
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LED.kLEDCount);

  public LEDs() {
    // Reuse buffer
    // Default to a length of 43, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    rainbow = LEDPattern.rainbow(255, 127).scrollAtRelativeSpeed(Percent.per(Second).of(50));

    // Apply the LED pattern to the data buffer
    rainbow.applyTo(m_ledBuffer);

    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);
  }
  
  @Override
  public void periodic() {
    // Apply the LED pattern to the data buffer
    rainbow.applyTo(m_ledBuffer);
    
    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);
  }
  
  @Override
  public void simulationPeriodic() {
  }
}