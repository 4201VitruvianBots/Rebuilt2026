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
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CLIMBER;
import frc.team4201.lib.utils.CtreUtils;

public class Climber extends SubsystemBase {

  // Creates a new motor object.
  @Logged(name = "Climber Motor", importance = Importance.DEBUG)
  private final TalonFX m_climberMotor = new TalonFX(CAN.kClimberMotor);

  private final DynamicMotionMagicVoltage m_request =
      new DynamicMotionMagicVoltage(
              0.0, CLIMBER.motionMagicCruiseVelocitynoRobot, CLIMBER.motionMagicAccelerationnoRobot)
          .withEnableFOC(true)
          .withSlot(0);

  // The position it's trying to reach and stabilise at.
  private Distance m_desiredPosition = Inches.of(0.0);

  @Logged(name = "Neutral Mode", importance = Logged.Importance.INFO)
  private NeutralModeValue m_neutralMode =
      NeutralModeValue.Brake; // Coast: you let go, gravity lets it fall. Brake: locks it in place.

  // Climber Sim State:
  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_ClimberSim =
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
    // configuring the motor for the elevator:
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kV = CLIMBER.kVnoRobot;
    config.Slot0.kA = CLIMBER.kAnoRobot;
    config.Slot0.kP = CLIMBER.kPnoRobot;
    config.Slot0.kD = CLIMBER.kDnoRobot;
    config.Slot0.kG = CLIMBER.kGnoRobot;
    config.Slot0.kS = CLIMBER.kS;

    config.Slot1.kV = CLIMBER.kVRobot;
    config.Slot1.kA = CLIMBER.kARobot;
    config.Slot1.kP = CLIMBER.kPRobot;
    config.Slot1.kD = CLIMBER.kDRobot;
    config.Slot1.kS = CLIMBER.kS;
    config.Slot1.kG = CLIMBER.kGRobot;

    config.Feedback.SensorToMechanismRatio =
        CLIMBER.gearRatio; // configNoRobotures climber to gear ratio. (check if absolute cancoder)
    config.MotionMagic.MotionMagicCruiseVelocity = CLIMBER.motionMagicCruiseVelocitynoRobot;
    config.MotionMagic.MotionMagicAcceleration = CLIMBER.motionMagicAccelerationnoRobot;
    // config.MotionMagic.MotionMagicJerk = CLIMBER.motionMagicJerk; // TODO: Implement Jerk when
    // needed.
    config.CurrentLimits.StatorCurrentLimit =
        40; // Prevents Climber from moving too jerkily and also breakage. TODO: Adjust this value.
    config.CurrentLimits.StatorCurrentLimitEnable = true; // Enables previous function.
    // Sets limits on motor output. Seperate from current limits.
    config.MotorOutput.PeakReverseDutyCycle = CLIMBER.peakReverseOutput;
    config.MotorOutput.PeakForwardDutyCycle = CLIMBER.peakForwardOutput;
    config.MotorOutput.NeutralMode = m_neutralMode; // Puts the motor in Neutral mode.

    // This is the function that applies all these configNoRoboturations to the motor.
    CtreUtils.configureTalonFx(m_climberMotor, config);

    m_motorSimState = m_climberMotor.getSimState();

    m_climberMotor.setPosition(Rotations.of(0));
  }

  @Logged(name = "Height Inches", importance = Logged.Importance.INFO)
  public double getHeightInches() {
    return getHeight().in(Inches);
  }

  @Logged(name = "Height Meters", importance = Logged.Importance.DEBUG)
  public Distance getHeight() {
    return CLIMBER.drumRotationsToDistance.times(
        m_climberMotor.getPosition().clone().refresh().getValue().magnitude());
  }

  public void setDesiredPositionAndMotionMagicConfigs(
      Distance desiredPosition,
      double MotionMagicVelocity,
      double MotionMagicAcceleration,
      double Jerk) {
    m_request.Velocity = MotionMagicVelocity;
    m_request.Acceleration = MotionMagicAcceleration;
    m_request.Jerk = Jerk;
    m_desiredPosition =
        Meters.of(
            MathUtil.clamp(
                desiredPosition.in(Meters),
                CLIMBER.lowerLimit.in(Meters),
                CLIMBER.upperLimit.in(Meters)));
    m_climberMotor.setControl(
        m_request.withPosition(
            m_desiredPosition.in(Meters) / CLIMBER.drumRotationsToDistance.in(Meters)));
    System.out.println(m_desiredPosition.in(Inches));
  }

  public void setPIDSlot(int slot) {
    m_request.Slot = slot;
  }

  public Current getStatorCurrent() {
    return m_climberMotor.getStatorCurrent().clone().refresh().getValue();
  }

  public void holdClimber() {
    setDesiredPositionAndMotionMagicConfigs(
        getHeight(),
        CLIMBER.motionMagicCruiseVelocityRobot,
        CLIMBER.motionMagicAccelerationRobot,
        0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_ClimberSim.setInputVoltage(m_motorSimState.getMotorVoltage());

    m_ClimberSim.update(0.020);

    m_motorSimState.setRawRotorPosition(
        m_ClimberSim.getPositionMeters()
            * CLIMBER.gearRatio
            / CLIMBER.drumRotationsToDistance.in(Meters));
    m_motorSimState.setRotorVelocity(
        m_ClimberSim.getVelocityMetersPerSecond()
            * CLIMBER.gearRatio
            / CLIMBER.drumRotationsToDistance.in(Meters));
  }
}
