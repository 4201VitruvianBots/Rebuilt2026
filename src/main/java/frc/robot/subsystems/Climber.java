// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.ROBOT.CONTROL_MODE;


public class Climber extends SubsystemBase {

  TalonFX m_climberMotor = new TalonFX(CAN.kClimberMotor); //Update Number if the DeviceID changes.

  // Booleans:
  private CONTROL_MODE m_controlMode = CONTROL_MODE.OPEN_LOOP;
  private double m_buttonInput = 0.0;

  /** Creates a new Climber. */
  public Climber() {
    //Configuring the motor for the elevator:
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kG = CLIMBER.kG;
    config.Slot0.kV = CLIMBER.kV;
    config.Slot0.kA = CLIMBER.kA;
    config.Slot0.kP = CLIMBER.kP;
    config.Slot0.kI = CLIMBER.kI;
    config.Slot0.kD = CLIMBER.kD;

    config.Feedback.SensorToMechanismRatio = CLIMBER.gearRatio;
  }

  //Methods
  public void setControlMode(CONTROL_MODE controlMode) {
    m_controlMode = controlMode;
  }

  public void setButtonInput(double buttonInput) {
    m_buttonInput = buttonInput;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
