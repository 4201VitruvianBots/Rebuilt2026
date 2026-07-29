// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Logged.Importance;
import org.wpilib.epilogue.NotLogged;
import org.wpilib.math.util.MathUtil;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.UPTAKE;
import frc.team4201.lib.utils.CtreUtils;

public class Uptake extends SubsystemBase {

  @Logged(name = "Uptake Motor", importance = Logged.Importance.DEBUG)
  private final TalonFX m_motor = new TalonFX(CAN.kUptakeMotor, CAN.roboRIO);

  private final FlywheelSim m_motorSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(UPTAKE.gearbox, UPTAKE.kInertia, UPTAKE.gearRatio),
          UPTAKE.gearbox);

  private final TalonFXSimState m_simState;

  private VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0.0);
  private DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0.0);

  private static AngularVelocity m_velocitySetpoint = RPM.of(0.0);

  public final DoubleSubscriber m_rpmSubscriber;
  public final DoublePublisher m_rpmPublisher;

  public Uptake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = UPTAKE.kP;
    config.Slot0.kS = UPTAKE.kS;
    config.Slot0.kV = UPTAKE.kV;
    config.CurrentLimits.StatorCurrentLimit = UPTAKE.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = UPTAKE.gearRatio;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    CtreUtils.configureTalonFx(m_motor, config);

    m_simState = m_motor.getSimState();
    var topic =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic("Uptake RPM Setpoint");
    m_rpmSubscriber = topic.subscribe(0.0);
    m_rpmPublisher = topic.publish();
  }

  public void setPercentOutput(double speed) {
    m_motor.setThrottle(speed);
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

  public boolean isShooting() {
    return getPercentOutput() > 0.01;
  }

  public double getPercentOutput() {
    return m_motor.getThrottle();
  }

  public double getRPMerror() {
    return getRPMsetpoint() - getMotorSpeedRPM();
  }

  @Logged(name = "Motor Velocity RPM", importance = Logged.Importance.INFO)
  public double getMotorSpeedRPM() {
    return m_motor.getVelocity().refresh().getValue().in(RPM);
  }

  public boolean isAtRPMsetpoint() {
    return getAbsoluteRPMerror() <= UPTAKE.kVelocityErrorThreshold;
  }

  public double getAbsoluteRPMerror() {
    return Math.abs(getRPMerror());
  }

  @NotLogged
  public Command percentCommand(double speed) {
    return this.startEnd(
        () -> m_motor.setThrottle(speed),
        () -> {
          setPercentOutput(0.0);
        });
  }

  public void testInit() {
    m_rpmPublisher.set(0.0);
  }

  public void testPeriodic() {
    setPercentOutput(m_rpmSubscriber.get());
  }

  @Override
  public void periodic() {
    // if (!isAtRPMsetpoint()) {
    //   m_motor.setControl(m_dutyCycleOut.withOutput(Math.signum(getRPMerror()) / 2));
    // } else {
    // m_motor.setControl(m_request.withVelocity(m_velocitySetpoint.abs(RotationsPerSecond)));
  }

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_motorSim.setInputVoltage(m_simState.getMotorVoltage());

    m_motorSim.update(0.02);

    m_simState.setRawRotorPosition(
        Rotations.of(m_motorSim.getAngularVelocity()).times(UPTAKE.gearRatio));
    m_simState.setRotorVelocity(RPM.of(m_motorSim.getAngularVelocity()).times(UPTAKE.gearRatio));
  }
}
