// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.ROBOT.CONTROL_MODE;
import frc.team4201.lib.utils.CtreUtils;


public class Climber extends SubsystemBase {

  //Creates new motor
  TalonFX m_climberMotor = new TalonFX(CAN.kClimberMotor);

  //For logging. It gets you the value of these 4 things. 
  private final StatusSignal<Angle> m_positionSignal = m_climberMotor.getPosition().clone();
  private final StatusSignal<AngularVelocity> m_velocitySignal = m_climberMotor.getVelocity().clone();
  private final StatusSignal<AngularAcceleration> m_accelSignal = m_climberMotor.getAcceleration().clone();
  private final StatusSignal<Voltage> m_voltageSignal = m_climberMotor.getMotorVoltage().clone();

  //The position it's trying to reach and stabilise at
  private Distance m_desiredPosition = Inches.of(0);

  //indicates how much the joystick is moving. -1 is all the way down, 1 is all the way up.
  @Logged(name = "Joystick Input", importance = Logged.Importance.DEBUG)
  private double m_joystickInput = 0.0;

  //open loop is bang bang control
  //closed loop is pid and motion magic
  @Logged(name = "Control Mode", importance = Logged.Importance.INFO)
  private CONTROL_MODE m_controlMode = CONTROL_MODE.OPEN_LOOP;

  //Coast: you let go, gravity lets it fall
  //Break: locks it in place
  @Logged(name = "Neutral Mode", importance = Logged.Importance.INFO)
  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake; //Maye set to coast?

  //Climber Sim State:
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

    config.Feedback.SensorToMechanismRatio = CLIMBER.gearRatio; //configures climber to gear ratio. (check if absolute cancoder)
    config.MotionMagic.MotionMagicCruiseVelocity = CLIMBER.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = CLIMBER.motionMagicAcceleration;
    config.CurrentLimits.StatorCurrentLimit = 40; //TODO: Adjust these
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.PeakReverseDutyCycle = CLIMBER.peakReverseOutput;
    config.MotorOutput.PeakForwardDutyCycle = CLIMBER.peakForwardOutput;
    config.MotorOutput.NeutralMode = m_neutralMode;

    CtreUtils.configureTalonFx(m_climberMotor, config); //Configures the motor.

    m_motorSimState = m_climberMotor.getSimState();

    m_climberMotor.setPosition(Rotations.of(0));

    setName("Climber");
    SmartDashboard.putData(this);
  }

  //Logging:
  //Logs the Status signals
  @Logged(name = "Motor Velocity", importance = Logged.Importance.DEBUG)
  public AngularVelocity getMotorVelocity() {
    return m_velocitySignal.refresh().getValue();
  }

    @Logged(name = "Motor Rotations", importance = Logged.Importance.DEBUG)
  public Angle getRotations() {
    return m_positionSignal.refresh().getValue();
  }

  @Logged(name = "Motor Acceleration", importance = Logged.Importance.DEBUG)
  public AngularAcceleration getMotorAcceleration() {
    return m_accelSignal.refresh().getValue();
  }

    @Logged(name = "Motor Voltage", importance = Logged.Importance.DEBUG)
  public Voltage getMotorVoltage() {
    return m_voltageSignal.refresh().getValue();
  }
  //End of logging

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
