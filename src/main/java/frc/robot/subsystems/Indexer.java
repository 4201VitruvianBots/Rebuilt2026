// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.INDEXER;
import frc.robot.constants.INDEXER.INDEXER_SPEED_1;
import frc.robot.constants.INDEXER.INDEXER_SPEED_2;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredDevice;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredSubsystem;
import frc.team4201.lib.simulation.TalonFXSim;
import frc.team4201.lib.utils.CtreUtils;

public class Indexer extends SubsystemBase implements MonitoredSubsystem {

  @MonitoredDevice(
      name = "Indexer Motor 1",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "rio")
  @Logged(name = "Indexer Motor 1", importance = Importance.INFO)
  private final TalonFX m_indexerMotor1 = new TalonFX(CAN.kIndexerMotor1, CAN.canivore);

  @MonitoredDevice(
      name = "Indexer Motor 2",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "rio")
  @Logged(name = "Indexer Motor 2", importance = Importance.INFO)
  private final TalonFX m_indexerMotor2 = new TalonFX(CAN.kIndexerMotor2, CAN.canivore);

  // @Logged(name = "Indexer Motor 3", importance = Importance.DEBUG)
  // private final TalonFX m_indexerMotor3 = new TalonFX(CAN.kIndexerMotor3);

  private DoubleSubscriber m_speedSubscriber1;
  private DoublePublisher m_speedPublisher1;
  private DoubleSubscriber m_speedSubscriber2;
  private DoublePublisher m_speedPublisher2;

  private final DCMotorSim m_indexerMotor1Sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(INDEXER.gearbox, INDEXER.kInertia, INDEXER.gearRatio),
          INDEXER.gearbox);
  private final TalonFXSim m_talonFxSim =
      new TalonFXSim(m_indexerMotor1Sim, m_indexerMotor1, m_indexerMotor2);

  /** Creates a new Indexer. */
  public Indexer() {}

  @Override
  public void initDevices() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = INDEXER.kP;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.PeakForwardDutyCycle = INDEXER.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = INDEXER.peakReverseOutput;
    config.Feedback.SensorToMechanismRatio = INDEXER.gearRatio;

    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    CtreUtils.configureDevice(m_indexerMotor1, config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CtreUtils.configureDevice(m_indexerMotor2, config);
  }

  public void setSpeeds(double speed1, double speed2) {
    m_indexerMotor1.set(speed1);
    m_indexerMotor2.set(speed2);
  }

  public boolean isConnected() {
    return m_indexerMotor1.isConnected() && m_indexerMotor2.isConnected();
  }

  @Logged(name = "Motor Output 1", importance = Logged.Importance.INFO)
  public double getPercentOutput() {
    return m_indexerMotor1.get();
  }

  @Logged(name = "Motor Output 2", importance = Logged.Importance.INFO)
  public double getPercentOutput2() {
    return m_indexerMotor2.get();
  }

  @NotLogged
  public Command command(INDEXER_SPEED_1 speed1, INDEXER_SPEED_2 speed2) {
    return this.startEnd(() -> setSpeeds(speed1.get(), speed2.get()), () -> setSpeeds(0.0, 0.0));
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_talonFxSim.update();
  }

  public void testInit() {
    var topic =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic("Indexer Roller Speed Setpoint 1");
    m_speedSubscriber1 = topic.subscribe(0.0);
    m_speedPublisher1 = topic.publish();
    var topic2 =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic("Indexer Roller Speed Setpoint 2");
    m_speedSubscriber2 = topic2.subscribe(0.0);
    m_speedPublisher2 = topic2.publish();
    m_speedPublisher1.set(0.0);
    m_speedPublisher2.set(0.0);
  }

  public void testPeriodic() {
    setSpeeds(m_speedSubscriber1.get(), m_speedSubscriber2.get());
  }
}
