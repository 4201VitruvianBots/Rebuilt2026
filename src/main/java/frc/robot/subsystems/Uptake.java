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
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.INTAKEMOTORS;
import frc.robot.Constants.UPTAKEMOTORS;
import frc.team4201.lib.utils.CtreUtils;

public class Uptake extends SubsystemBase {

  private final TalonFX m_motor = new TalonFX(CAN.kUptakeMotor);

  private final DCMotorSim m_motorSim = 
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              UPTAKEMOTORS.gearbox, UPTAKEMOTORS.gearRatio, UPTAKEMOTORS.kInertia),
              UPTAKEMOTORS.gearbox);

  private final TalonFXSimState m_simState;

  /** Creates a new Uptake. */
  public Uptake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = UPTAKEMOTORS.kP;
    config.Slot0.kD = UPTAKEMOTORS.kD;
    config.Slot0.kV = UPTAKEMOTORS.kV;
    config.Feedback.SensorToMechanismRatio = UPTAKEMOTORS.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CtreUtils.configureTalonFx(m_motor, config);

    m_simState = m_motor.getSimState();
  }

  public void setPercentOutput(double speed) {
    m_motor.set(speed);
  }

  public boolean isConnected() {
    return m_motor.isConnected();
  }

  @Logged(name = "Motor Output %", importance = Logged.Importance.INFO)
  public double getPercentOutput() {
    return m_motor.get();
  }

  @Override
  public void periodic() {}

      @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_motorSim.setInputVoltage(m_simState.getMotorVoltage());

    m_motorSim.update(0.02);

    m_simState.setRawRotorPosition(
        Rotations.of(m_motorSim.getAngularPositionRotations())
            .times(INTAKEMOTORS.ROLLERS.gearRatio));
    m_simState.setRotorVelocity(
        RPM.of(m_motorSim.getAngularVelocityRPM()).times(INTAKEMOTORS.ROLLERS.gearRatio));
  }
}
