// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.INTAKEMOTORS;
import frc.team4201.lib.utils.CtreUtils;

public class Intake extends SubsystemBase {

  @Logged(name = "Intake Motor 1", importance = Logged.Importance.DEBUG)
  private final TalonFX m_motor1 = new TalonFX(CAN.kIntakeRollerMotor1);

  // private final TalonFX m_motor2 = new TalonFX(CAN.kIntakeRollerMotor2);

  private final DCMotorSim m_motor1Sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              INTAKEMOTORS.ROLLERS.gearbox,
              INTAKEMOTORS.ROLLERS.gearRatio,
              INTAKEMOTORS.ROLLERS.kInertia),
          INTAKEMOTORS.ROLLERS.gearbox);

  private final TalonFXSimState m_simState;

  /** Creates a new Intake. */
  public Intake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = INTAKEMOTORS.ROLLERS.kP;
    config.Feedback.SensorToMechanismRatio = INTAKEMOTORS.ROLLERS.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CtreUtils.configureTalonFx(m_motor1, config);
    // CtreUtils.configureTalonFx(m_motor2, config);

    // m_motor2.setControl(new Follower(m_motor1.getDeviceID(), MotorAlignmentValue.Opposed));

    m_simState = m_motor1.getSimState();
  }

  public void setOutputPercent(double speed) {
    m_motor1.set(speed);
  }

  public boolean isConnected() {
    return m_motor1.isConnected(); // && m_motor2.isConnected();
  }

  @Logged(name = "Motor Output %", importance = Logged.Importance.INFO)
  public double getPercentOutput() {
    return m_motor1.get();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_motor1Sim.setInputVoltage(m_simState.getMotorVoltage());

    m_motor1Sim.update(0.02);

    m_simState.setRawRotorPosition(
        Rotations.of(m_motor1Sim.getAngularPositionRotations())
            .times(INTAKEMOTORS.ROLLERS.gearRatio));
    m_simState.setRotorVelocity(
        RPM.of(m_motor1Sim.getAngularVelocityRPM()).times(INTAKEMOTORS.ROLLERS.gearRatio));
  }
}
