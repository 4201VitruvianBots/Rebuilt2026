// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.ROBOT.CONTROL_MODE;


public class Climber extends SubsystemBase {

  TalonFX m_climberMotor = new TalonFX(CAN.kClimberMotor); //Update Number if the DeviceID changes.

  // Booleans:
  
  //Logging
  @Logged(name = "Neutral Mode", importance = Logged.Importance.INFO)
  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake; //Maye set to coast?

  /** Creates a new Climber. */
  public Climber() {
    //Configuring the motor for the elevator:
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kV = CLIMBER.kV;
    config.Slot0.kA = CLIMBER.kA;
    config.Slot0.kP = CLIMBER.kP;

    config.Slot0.kD = CLIMBER.kD;

    config.Feedback.SensorToMechanismRatio = CLIMBER.gearRatio; //configures climber to gear ratio. (check if absolute cancoder)
    config.MotionMagic.MotionMagicCruiseVelocity = CLIMBER.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = CLIMBER.motionMagicAcceleration;
    config.CurrentLimits.StatorCurrentLimit = 40; //TODO: Adjust these
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.PeakReverseDutyCycle = CLIMBER.peakReverseOutput;
    config.MotorOutput.PeakForwardDutyCycle = CLIMBER.peakForwardOutput;
    config.MotorOutput.NeutralMode = m_neutralMode;
  }

  //Methods

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
