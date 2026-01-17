// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.INDEXERMOTORS;
import frc.team4201.lib.utils.CtreUtils;

public class Indexer extends SubsystemBase {

  private final TalonFX[] m_indexerMotors = {
    new TalonFX(CAN.kIndexerMotor1),
    new TalonFX(CAN.kIndexerMotor2),
    new TalonFX(CAN.kIndexerMotor3)
  };
  private NeutralModeValue m_neutralMode = 
      NeutralModeValue.Brake;
    private final MotionMagicVelocityVoltage m_request = 
      new MotionMagicVelocityVoltage(0).withEnableFOC(true).withVelocity(0.6);
    private double m_setSpeed;

        private final DCMotorSim m_indexerMotorSim =
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
    config.Slot0.kI = INDEXERMOTORS.kI;
    config.Slot0.kD = INDEXERMOTORS.kD;

    config.MotorOutput.NeutralMode = m_neutralMode;
    config.MotorOutput.PeakForwardDutyCycle = INDEXERMOTORS.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = INDEXERMOTORS.peakReverseOutput;
    config.Feedback.SensorToMechanismRatio = INDEXERMOTORS.gearRatio;

    config.MotionMagic.MotionMagicAcceleration = INDEXERMOTORS.motionMagicAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = INDEXERMOTORS.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicJerk = INDEXERMOTORS.motionMagicJerk;

    for (int i = 0; i < m_indexerMotors.length; i++ ) {
      CtreUtils.configureTalonFx(m_indexerMotors[i], config);
    }

    for (int i = 2; i < m_indexerMotors.length; i++) {
      m_indexerMotors[i].setControl(
          new Follower(
              m_indexerMotors[0].getDeviceID(),
              MotorAlignmentValue
                  .Aligned));
    }
    m_indexerMotors[1].setControl(new Follower(m_indexerMotors[0].getDeviceID(), MotorAlignmentValue.Opposed));

    m_simState =
      m_indexerMotors[0]
              .getSimState();
    
  }
  public void setPercentOutputFOC(double speed) {
    m_setSpeed = speed;
    m_indexerMotors[0].set(speed);
  }

  

  @Logged(name = "Motor Output",importance =  Logged.Importance.INFO)
  public double getPercentOutput() {
    return m_indexerMotors[0].get();
  }

  @Logged(name = "Motor Speed", importance = Logged.Importance.DEBUG)
  public AngularVelocity getMotorSpeed() {
    return m_indexerMotors[0].getVelocity().refresh().getValue();
  }
  @Logged(name = "Motor Voltage", importance =  Logged.Importance.DEBUG)
  public Voltage getMotorVoltage() {
    return m_indexerMotors[0].getMotorVoltage().refresh().getValue();
  }

 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_indexerMotorSim.setInputVoltage(m_simState.getMotorVoltage());

    m_indexerMotorSim.update(0.02);

    m_simState.setRawRotorPosition(
      Rotations.of(m_indexerMotorSim.getAngularPositionRotations())
        .times(INDEXERMOTORS.gearRatio));
    m_simState.setRotorVelocity(
      RPM.of(m_indexerMotorSim.getAngularVelocityRPM())
        .times(INDEXERMOTORS.gearRatio));
  }
}
