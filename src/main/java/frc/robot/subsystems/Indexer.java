// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.robot.constants.CAN;
import frc.robot.constants.INDEXER;
import frc.robot.constants.INDEXER.INDEXER_SPEED_1;
import frc.team4201.lib.utils.CtreUtils;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Logged.Importance;
import org.wpilib.epilogue.NotLogged;
import org.wpilib.math.system.Models;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.RobotController;

public class Indexer extends SubsystemBase {

  @Logged(name = "Indexer Motor 1", importance = Importance.INFO)
  private final TalonFX m_indexerMotor1 = new TalonFX(CAN.kIndexerMotor1, CAN.S1);

  @Logged(name = "Indexer Motor 2", importance = Importance.INFO)
  private final TalonFX m_indexerMotor2 = new TalonFX(CAN.kIndexerMotor2, CAN.S1);

  @Logged(name = "Indexer Motor 3", importance = Importance.INFO)
  private final TalonFX m_indexerMotor3 = new TalonFX(CAN.kIndexerMotor3, CAN.S3);

  @Logged(name = "Indexer Motor 4", importance = Importance.INFO)
  private final TalonFX m_indexerMotor4 = new TalonFX(CAN.kIndexerMotor3, CAN.S3);

  private DoubleSubscriber m_speedSubscriber1;
  private DoublePublisher m_speedPublisher1;

  private final DCMotorSim m_indexerMotor1Sim =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(
              INDEXER.gearbox, INDEXER.kInertia, INDEXER.gearRatio),
          INDEXER.gearbox);

  private final TalonFXSimState m_simState1;

  /** Creates a new Indexer. */
  public Indexer() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.PeakForwardDutyCycle = INDEXER.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = INDEXER.peakReverseOutput;

    config.CurrentLimits.StatorCurrentLimit = INDEXER.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    CtreUtils.configureTalonFx(m_indexerMotor1, config);
    CtreUtils.configureTalonFx(m_indexerMotor2, config);
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CtreUtils.configureTalonFx(m_indexerMotor3, config);
    CtreUtils.configureTalonFx(m_indexerMotor4, config);

    m_indexerMotor2.setControl(
        new Follower(m_indexerMotor1.getDeviceID(), MotorAlignmentValue.Aligned));
    m_indexerMotor4.setControl(
        new Follower(m_indexerMotor3.getDeviceID(), MotorAlignmentValue.Aligned));

    m_simState1 = m_indexerMotor1.getSimState();
  }

  public void setSpeed(double speed) {
    m_indexerMotor1.setThrottle(speed);
    m_indexerMotor3.setThrottle(speed);
  }

  public boolean isConnected() {
    return m_indexerMotor1.isConnected() && m_indexerMotor3.isConnected();
  }

  @Logged(name = "Motor Output S1", importance = Logged.Importance.DEBUG)
  public double getS1PercentOutput() {
    return m_indexerMotor1.getThrottle();
  }

  @NotLogged
  public Command command(INDEXER_SPEED_1 speed1) {
    return this.startEnd(() -> setSpeed(speed1.get()), () -> setSpeed(0.0));
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_simState1.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_indexerMotor1Sim.setInputVoltage(m_simState1.getMotorVoltage());

    m_indexerMotor1Sim.update(0.02);

    m_simState1.setRawRotorPosition(
        Rotations.of(m_indexerMotor1Sim.getAngularPosition()).times(INDEXER.gearRatio));
    m_simState1.setRotorVelocity(
        RPM.of(m_indexerMotor1Sim.getAngularVelocity()).times(INDEXER.gearRatio));
  }

  public void utilityInit() {
    var topic =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic("Indexer Roller Speed Setpoint 1");
    m_speedSubscriber1 = topic.subscribe(0.0);
    m_speedPublisher1 = topic.publish();
    m_speedPublisher1.set(0.0);
  }

  public void utilityPeriodic() {
    setSpeed(m_speedSubscriber1.get());
  }
}
