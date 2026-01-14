// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SHOOTERHOOD;
import frc.robot.Constants.SHOOTERHOOD.HoodAngle;
import frc.robot.Constants.SHOOTERMOTORS;
import frc.team4201.lib.utils.CtreUtils;

public class ShooterHood extends SubsystemBase {

  @Logged(name = "Hood Motor", importance = Importance.DEBUG)
  private final TalonFX m_motor =
      new TalonFX(CAN.kShooterHoodMotor); // Replace these device ids after motors are set up

  private NeutralModeValue m_neutralMode =
      NeutralModeValue.Brake; // Brake... because this is a hood. That doesn't coast.
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0).withEnableFOC(true);
  private Angle m_hoodSetpoint = HoodAngle.CLOSE.getAngle();

  private final DCMotorSim m_shooterHoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              SHOOTERHOOD.gearbox, SHOOTERHOOD.kInertia, SHOOTERHOOD.gearRatio),
          SHOOTERHOOD.gearbox);
  private final TalonFXSimState m_simState;

  public ShooterHood() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = SHOOTERHOOD.kP;
    config.Slot0.kI = SHOOTERHOOD.kI;
    config.Slot0.kD = SHOOTERHOOD.kD;
    // config.Slot0.kA = SHOOTERHOOD.kA;
    // config.Slot0.kV = SHOOTERHOOD.kV;
    config.MotorOutput.NeutralMode = m_neutralMode;
    config.Feedback.SensorToMechanismRatio = SHOOTERHOOD.gearRatio;
    config.MotorOutput.PeakForwardDutyCycle = SHOOTERHOOD.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = SHOOTERHOOD.peakReverseOutput;
    config.CurrentLimits.StatorCurrentLimit = 30;

    config.MotionMagic.MotionMagicCruiseVelocity = SHOOTERHOOD.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = SHOOTERHOOD.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = SHOOTERHOOD.motionMagicJerk;

    CtreUtils.configureTalonFx(m_motor, config);

    m_simState = m_motor.getSimState();
  }

  public void setShooterHoodSetpoint(Angle setpoint) {
    m_hoodSetpoint = setpoint;
    m_motor.setControl(m_request.withPosition(setpoint));
  }

  public Angle getShooterHoodSetpoint() {
    return m_hoodSetpoint;
  }

  @Logged(name = "Motor Voltage", importance = Importance.DEBUG)
  public Voltage getHoodVoltage() {
    return m_motor.getMotorVoltage().refresh().getValue();
  }

  @Logged(name = "Motor Velocity", importance = Importance.DEBUG)
  public AngularVelocity getHoodVelocity() {
    return m_motor.getVelocity().refresh().getValue();
  }

  @Logged(name = "Hood Angle", importance = Importance.INFO)
  public double getHoodAngle() {
    return m_motor.getPosition().refresh().getValueAsDouble() * 360;
  }

  @Logged(name = "Hood Rotations", importance = Importance.DEBUG)
  public Angle getHoodRotations() {
    return m_motor.getPosition().refresh().getValue();
  }

  @Logged(name = "At Setpoint", importance = Logged.Importance.DEBUG)
  public boolean atSetpoint() {
    return m_hoodSetpoint.minus(getHoodRotations()).abs(Degrees) <= 1; // RIP the 254 reference
  }

  public boolean[] isConnected() {
    return new boolean[] {m_motor.isConnected()};
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_shooterHoodSim.setInputVoltage(m_simState.getMotorVoltage());

    m_shooterHoodSim.update(0.02);

    m_simState.setRawRotorPosition(
        Rotations.of(m_shooterHoodSim.getAngularPositionRotations())
            .times(SHOOTERMOTORS.gearRatio));
    m_simState.setRotorVelocity(
        RPM.of(m_shooterHoodSim.getAngularVelocityRPM()).times(SHOOTERMOTORS.gearRatio));
  }
}
