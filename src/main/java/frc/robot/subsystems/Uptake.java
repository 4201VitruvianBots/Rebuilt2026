// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.UPTAKE;
import frc.team4201.lib.utils.CtreUtils;

public class Uptake extends SubsystemBase {

  @Logged(name = "Uptake Motor", importance = Logged.Importance.DEBUG)
  private final TalonFX m_motor = new TalonFX(CAN.kUptakeMotor, CAN.driveBaseCanbus);

  private final FlywheelSim m_motorSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(UPTAKE.gearbox, UPTAKE.kInertia, UPTAKE.gearRatio),
          UPTAKE.gearbox);

  private final TalonFXSimState m_simState;

  private MotionMagicVelocityTorqueCurrentFOC m_request =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  private static AngularVelocity m_velocitySetpoint = RPM.of(0.0);

  public Uptake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = UPTAKE.kP;
    config.Slot0.kS = UPTAKE.kS;
    config.Slot0.kV = UPTAKE.kV;

    config.Feedback.SensorToMechanismRatio = UPTAKE.gearRatio;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.MotionMagic.MotionMagicAcceleration = UPTAKE.kMotionMagicAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = UPTAKE.kMotionMagicCruiseVelocity;

    CtreUtils.configureTalonFx(m_motor, config);

    m_simState = m_motor.getSimState();
  }

  public void setPercentOutput(double speed) {
    m_motor.set(speed);
  }

  // Epilogue doesn't log RPM correctly so it must be a double
  @Logged(name = "RPM Setpoint", importance = Importance.DEBUG)
  public double getRPMsetpoint() {
    return m_velocitySetpoint.in(RPM);
  }

  public void setVelocitySetpoint(AngularVelocity setpoint) {
    m_velocitySetpoint =
        RPM.of(MathUtil.clamp(setpoint.in(RPM), UPTAKE.minRPM.in(RPM), UPTAKE.maxRPM.in(RPM)));
  }

  public boolean isConnected() {
    return m_motor.isConnected();
  }

  public double getPercentOutput() {
    return m_motor.get();
  }

  @Logged(name = "Motor Velocity RPM", importance = Logged.Importance.INFO)
  public double getMotorSpeedRPM() {
    return m_motor.getVelocity().refresh().getValue().in(RPM);
  }

  @Override
  public void periodic() {
    m_motor.setControl(m_request.withVelocity(m_velocitySetpoint.abs(RotationsPerSecond)));
  }

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_motorSim.setInputVoltage(m_simState.getMotorVoltage());

    m_motorSim.update(0.02);

    m_simState.setRawRotorPosition(
        Rotations.of(m_motorSim.getAngularVelocityRPM()).times(UPTAKE.gearRatio));
    m_simState.setRotorVelocity(RPM.of(m_motorSim.getAngularVelocityRPM()).times(UPTAKE.gearRatio));
  }
}
