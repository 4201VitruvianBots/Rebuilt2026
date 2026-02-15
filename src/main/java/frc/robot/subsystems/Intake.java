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
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.INTAKE;
import frc.robot.constants.INTAKE.ROLLERS.INTAKE_SPEED;
import frc.team4201.lib.utils.CtreUtils;

public class Intake extends SubsystemBase {

  @Logged(name = "Intake Motor", importance = Logged.Importance.DEBUG)
  private final TalonFX m_motor = new TalonFX(CAN.kIntakeRollerMotor1, CAN.driveBaseCanbus);

  private DoubleSubscriber m_outputSubscriber;
  private DoublePublisher m_outputPublisher;

  // private final TalonFX m_motor2 = new TalonFX(CAN.kIntakeRollerMotor2);

  private final DCMotorSim m_motor1Sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              INTAKE.ROLLERS.gearbox, INTAKE.ROLLERS.gearRatio, INTAKE.ROLLERS.kInertia),
          INTAKE.ROLLERS.gearbox);

  private final TalonFXSimState m_simState;

  /** Creates a new Intake. */
  public Intake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = INTAKE.ROLLERS.kP;
    config.Feedback.SensorToMechanismRatio = INTAKE.ROLLERS.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.PeakForwardDutyCycle = INTAKE.ROLLERS.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = INTAKE.ROLLERS.peakReverseOutput;

    CtreUtils.configureTalonFx(m_motor, config);
    // CtreUtils.configureTalonFx(m_motor2, config);

    // m_motor2.setControl(new Follower(m_motor.getDeviceID(), MotorAlignmentValue.Opposed));

    m_simState = m_motor.getSimState();
  }

  public void setOutputPercent(double speed) {
    m_motor.set(speed);
  }

  public boolean isConnected() {
    return m_motor.isConnected(); // && m_motor2.isConnected();
  }

  @Logged(name = "Motor Output %", importance = Logged.Importance.INFO)
  public double getPercentOutput() {
    return m_motor.get();
  }

  // For Robot2d simulation
  @NotLogged
  public AngularVelocity getVelocity() {
    return m_motor.getVelocity().clone().refresh().getValue();
  }

  @NotLogged
  public boolean isIntaking() {
    return m_motor.get() != 0;
  }

  @NotLogged
  public Command command(INTAKE_SPEED speed) {
    return this.startEnd(() -> m_motor.set(speed.get()), () -> m_motor.set(0));
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_motor1Sim.setInputVoltage(m_simState.getMotorVoltage());

    m_motor1Sim.update(0.02);

    m_simState.setRawRotorPosition(
        Rotations.of(m_motor1Sim.getAngularPositionRotations()).times(INTAKE.ROLLERS.gearRatio));
    m_simState.setRotorVelocity(
        RPM.of(m_motor1Sim.getAngularVelocityRPM()).times(INTAKE.ROLLERS.gearRatio));
  }

  public void testInit() {
    var topic =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic("Intake Roller Output Setpoint");
    m_outputSubscriber = topic.subscribe(0.0);
    m_outputPublisher = topic.publish();
    m_outputPublisher.set(0.0);
  }

  public void testPeriodic() {
    setOutputPercent(m_outputSubscriber.get());
  }
}
