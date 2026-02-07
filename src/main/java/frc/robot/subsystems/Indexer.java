// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.INDEXER;
import frc.team4201.lib.utils.CtreUtils;

public class Indexer extends SubsystemBase {

  @Logged(name = "Indexer Motor 1", importance = Importance.INFO)
  private final TalonFX m_indexerMotor1 = new TalonFX(CAN.kIndexerMotor1, CAN.driveBaseCanbus);

  @Logged(name = "Indexer Motor 2", importance = Importance.INFO)
  private final TalonFX m_indexerMotor2 = new TalonFX(CAN.kIndexerMotor2, CAN.driveBaseCanbus);

  // @Logged(name = "Indexer Motor 3", importance = Importance.DEBUG)
  // private final TalonFX m_indexerMotor3 = new TalonFX(CAN.kIndexerMotor3);

  private DoubleSubscriber m_speedSubscriber;
  private DoublePublisher m_speedPublisher;

  private final DCMotorSim m_indexerMotor1Sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(INDEXER.gearbox, INDEXER.kInertia, INDEXER.gearRatio),
          INDEXER.gearbox);
  private final DCMotorSim m_indexerMotor2Sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(INDEXER.gearbox, INDEXER.kInertia, INDEXER.gearRatio),
          INDEXER.gearbox);
  private final TalonFXSimState m_simState1;
  private final TalonFXSimState m_simState2;

  /** Creates a new Indexer. */
  public Indexer() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = INDEXER.kP;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.PeakForwardDutyCycle = INDEXER.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = INDEXER.peakReverseOutput;
    config.Feedback.SensorToMechanismRatio = INDEXER.gearRatio;

    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    CtreUtils.configureTalonFx(m_indexerMotor1, config);
    CtreUtils.configureTalonFx(m_indexerMotor2, config);

    // m_indexerMotor2.setControl(
    //     new Follower(m_indexerMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    // m_indexerMotor3.setControl(
    //     new Follower(m_indexerMotor1.getDeviceID(), MotorAlignmentValue.Aligned));

    m_simState1 = m_indexerMotor1.getSimState();
    m_simState2 = m_indexerMotor2.getSimState();


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

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_simState1.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_indexerMotor1Sim.setInputVoltage(m_simState1.getMotorVoltage());

    m_indexerMotor1Sim.update(0.02);

    m_simState1.setRawRotorPosition(
        Rotations.of(m_indexerMotor1Sim.getAngularPositionRotations()).times(INDEXER.gearRatio));
    m_simState1.setRotorVelocity(
        RPM.of(m_indexerMotor1Sim.getAngularVelocityRPM()).times(INDEXER.gearRatio));
    m_simState2.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_indexerMotor2Sim.setInputVoltage(m_simState1.getMotorVoltage());

    m_indexerMotor2Sim.update(0.02);

    m_simState2.setRawRotorPosition(
        Rotations.of(m_indexerMotor2Sim.getAngularPositionRotations()).times(INDEXER.gearRatio));
    m_simState2.setRotorVelocity(
        RPM.of(m_indexerMotor2Sim.getAngularVelocityRPM()).times(INDEXER.gearRatio));
  }

  public void testInit() {
        var topic =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic("Indexer Roller Speed Setpoint");
    m_speedSubscriber = topic.subscribe(0.0);
    m_speedPublisher = topic.publish();
    m_speedPublisher.set(0.0);
  }

  public void testPeriodic() {
    setSpeeds(m_speedSubscriber.get(), m_speedSubscriber.get());
  }
}
