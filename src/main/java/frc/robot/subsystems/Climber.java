// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports:
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.ROBOT.CONTROL_MODE;
import frc.team4201.lib.utils.CtreUtils;



public class Climber extends SubsystemBase {

  // Creates a new motor object.
  TalonFX m_climberMotor = new TalonFX(CAN.kClimberMotor);

  // Motion Magic things?
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0).withEnableFOC(true);


  // For logging. This is a custom command that logs important things for debugging. 
  @Logged (name="Climber Motor", importance = Importance.DEBUG)
  
  // The position it's trying to reach and stabilise at.
  private Distance m_desiredPosition = Inches.of(0);

  // Indicates how much the joystick is moving. -1 is all the way down, 1 is all the way up.
  @Logged(name = "Joystick Input", importance = Logged.Importance.DEBUG)
  private double m_joystickInput = 0.0;
  @Logged(name = "Control Mode", importance = Logged.Importance.INFO)
  private CONTROL_MODE m_controlMode = CONTROL_MODE.OPEN_LOOP; // Open loop is bang bang control. Closed loop is pid and motion magic.
  @Logged(name = "Neutral Mode", importance = Logged.Importance.INFO)
  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake; // Coast: you let go, gravity lets it fall. Break: locks it in place.

  // Climber Sim State:
  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          CLIMBER.gearbox,
          CLIMBER.gearRatio,
          CLIMBER.kCarriageMass.in(Kilograms),
          CLIMBER.kClimberDrumDiameter.div(2).in(Meters),
          CLIMBER.lowerLimit.in(Meters),
          CLIMBER.upperLimit.in(Meters),
          true,
          1);
  private final TalonFXSimState m_motorSimState;


  /** Creates a new Climber. */
  public Climber() {
    //Configuring the motor for the elevator:
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kV = CLIMBER.kV;
    config.Slot0.kA = CLIMBER.kA;
    config.Slot0.kP = CLIMBER.kP;
    config.Slot0.kD = CLIMBER.kD;

    config.Feedback.SensorToMechanismRatio = CLIMBER.gearRatio; // Configures climber to gear ratio. (check if absolute cancoder)
    config.MotionMagic.MotionMagicCruiseVelocity = CLIMBER.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = CLIMBER.motionMagicAcceleration;
    config.CurrentLimits.StatorCurrentLimit = 40; // Prevents Climber from moving too jerkily and also breakage. TODO: Adjust this value.
    config.CurrentLimits.StatorCurrentLimitEnable = true; //Enables preious function.
    // Sets limits on motor output. Seperate from current limits.
    config.MotorOutput.PeakReverseDutyCycle = 
        CLIMBER.peakReverseOutput;
    config.MotorOutput.PeakForwardDutyCycle = 
        CLIMBER.peakForwardOutput;
    config.MotorOutput.NeutralMode = m_neutralMode; // Puts the motor in Neutral mode.

    //This is the function that applies all these configurations to the motor.
    CtreUtils.configureTalonFx(m_climberMotor, config); 

    m_motorSimState = m_climberMotor.getSimState();

    m_climberMotor.setPosition(Rotations.of(0));

    setName("Climber");
    SmartDashboard.putData(this);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
