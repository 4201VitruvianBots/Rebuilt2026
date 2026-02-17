// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED.LED_STATES;

public class LEDs extends SubsystemBase {
  //TODO: test and setup these on roborio to allow LED changing
  DigitalOutput ledMode0 = new DigitalOutput(9);  
  DigitalOutput ledMode1 = new DigitalOutput(8);  
  DigitalOutput ledMode2 = new DigitalOutput(7);
  DigitalOutput ledMode3 = new DigitalOutput(6);

  private LED_STATES currentState = LED_STATES.DISABLED;

  public LEDs() {

  }

  public void setState(LED_STATES newState) {
    if (newState != currentState) {
      currentState = newState;

      ledMode0.set(newState.getBit(0));
      ledMode1.set(newState.getBit(1));
      ledMode2.set(newState.getBit(2));
      ledMode3.set(newState.getBit(3));

      System.out.println("LEDs: Changing state to " + newState.getAnimation());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
