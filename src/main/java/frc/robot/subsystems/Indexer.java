// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.INDEXERMOTORS;
import frc.team4201.lib.utils.CtreUtils;

public class Indexer extends SubsystemBase {

  @Logged(name = "Indexer Motor", importance = Importance.DEBUG)
  private final TalonFX m_indexerMotor1 = new TalonFX(CAN.kIndexerMotor1);
  @Logged(name = "Indexer Motor 2", importance = Importance.DEBUG)
  private final TalonFX m_indexerMotor2 = new TalonFX(CAN.kIndexerMotor2);
  @Logged(name = "Indexer Motor 3", importance = Importance.DEBUG)
  private final TalonFX m_indexerMotor3 = new TalonFX(CAN.kIndexerMotor3);

  private final StatusSignal<AngularVelocity> m_velocitySignal =
      m_indexerMotor1.getVelocity().clone();
  private final StatusSignal<Voltage> m_voltageSignal =
      m_indexerMotor1.getMotorVoltage().clone();
  private final StatusSignal<Current> m_supplyCurrentSignal =
      m_indexerMotor1.getSupplyCurrent().clone();
  private final StatusSignal<Current> m_statorCurrentSignal =
      m_indexerMotor1.getStatorCurrent().clone();
  private final StatusSignal<Current> m_torqueCurrentSignal =
      m_indexerMotor1.getTorqueCurrent().clone();

    private final DCMotorSim m_indexerMotor1Sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              INDEXERMOTORS.gearbox, INDEXERMOTORS.kInertia, INDEXERMOTORS.gearRatio),
          INDEXERMOTORS.gearbox);
  private final TalonFXSimState m_simState;
  /** Creates a new Indexer. */
  public Indexer() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kA = INDEXERMOTORS.kA;
    config.Slot0.kV = INDEXERMOTORS.kV;
    config.Slot0.kP = INDEXERMOTORS.kP;
    config.Slot0.kD = INDEXERMOTORS.kD;
    config.Slot0.kS = INDEXERMOTORS.kS;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.PeakForwardDutyCycle = INDEXERMOTORS.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = INDEXERMOTORS.peakReverseOutput;
    config.Feedback.SensorToMechanismRatio = INDEXERMOTORS.gearRatio;
    config.CurrentLimits.StatorCurrentLimit = 30;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    CtreUtils.configureTalonFx(m_indexerMotor1, config);

    m_indexerMotor2.setControl(new Follower(m_indexerMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    m_indexerMotor3.setControl(new Follower(m_indexerMotor1.getDeviceID(), MotorAlignmentValue.Aligned));

    m_simState =
      m_indexerMotor1
              .getSimState();
    
  }

  public void setSpeed(double speed) {
    m_indexerMotor1.set(speed);
  }


  @Logged(name = "Motor Output",importance =  Logged.Importance.INFO)
  public double getPercentOutput() {
    return m_indexerMotor1.get();
  }

  public AngularVelocity getMotorSpeed() {
    return m_velocitySignal.refresh().getValue();
  }

  public Voltage getMotorVoltage() {
    return m_voltageSignal.refresh().getValue();
  }
  
  public Current getSupplyCurrent() {
    return m_supplyCurrentSignal.refresh().getValue();
  }

  public Current getStatorCurrent() {
    return m_statorCurrentSignal.refresh().getValue();
  }

  public Current getTorqueCurrent() {
    return m_torqueCurrentSignal.refresh().getValue();
  }

  public boolean isConnected() {
    return m_indexerMotor1.isConnected();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_indexerMotor1Sim.setInputVoltage(m_simState.getMotorVoltage());

    m_indexerMotor1Sim.update(0.02);

    m_simState.setRawRotorPosition(
      Rotations.of(m_indexerMotor1Sim.getAngularPositionRotations())
        .times(INDEXERMOTORS.gearRatio));
    m_simState.setRotorVelocity(
      RPM.of(m_indexerMotor1Sim.getAngularVelocityRPM())
        .times(INDEXERMOTORS.gearRatio));
  }
}
