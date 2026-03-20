// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.INTAKE;
import frc.robot.constants.INTAKE.ROLLERS.INTAKE_SPEED;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredDevice;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredSubsystem;
import frc.team4201.lib.simulation.TalonFXSim;
import frc.team4201.lib.utils.CtreUtils;

public class Intake extends SubsystemBase implements MonitoredSubsystem {

  @MonitoredDevice(
      name = "Intake Motor",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "rio")
  @Logged(name = "Intake Motor", importance = Logged.Importance.DEBUG)
  private final TalonFX m_motor = new TalonFX(CAN.kIntakeRollerMotor1, CAN.canivore);

  private DoubleSubscriber m_outputSubscriber;
  private DoublePublisher m_outputPublisher;

  // private final TalonFX m_motor2 = new TalonFX(CAN.kIntakeRollerMotor2);

  private final DCMotorSim m_motor1Sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              INTAKE.ROLLERS.gearbox, INTAKE.ROLLERS.gearRatio, INTAKE.ROLLERS.kInertia),
          INTAKE.ROLLERS.gearbox);
  private final TalonFXSim m_talonFxSim =
      new TalonFXSim(m_motor1Sim, m_motor).withConversionFactor(INTAKE.ROLLERS.gearRatio);
  private int simStoredFuel = 8; // For fuel sim - 8 preload during auto

  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void initDevices() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = INTAKE.ROLLERS.kP;
    config.Feedback.SensorToMechanismRatio = INTAKE.ROLLERS.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.PeakForwardDutyCycle = INTAKE.ROLLERS.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = INTAKE.ROLLERS.peakReverseOutput;

    CtreUtils.configureDevice(m_motor, config);
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
    return m_motor.getVelocity().getValue();
  }

  @NotLogged
  public boolean isIntaking() {
    return m_motor.get() != 0;
  }

  @NotLogged
  public Command command(INTAKE_SPEED speed) {
    return this.startEnd(() -> m_motor.set(speed.get()), () -> m_motor.set(0));
  }

  @NotLogged
  public int getStoredFuel() {
    if (RobotBase.isSimulation()) {
      return simStoredFuel;
    } else {
      throw new UnsupportedOperationException("Attempted to get fuel sim count on real robot");
    }
  }

  @NotLogged
  public void setStoredFuel(int fuel) {
    if (RobotBase.isSimulation()) {
      simStoredFuel = fuel;
    } else {
      throw new UnsupportedOperationException("Attempted to set fuel sim count on real robot");
    }
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
            .getDoubleTopic("Intake Roller Output Setpoint");
    m_outputSubscriber = topic.subscribe(0.0);
    m_outputPublisher = topic.publish();
    m_outputPublisher.set(0.0);
  }

  public void testPeriodic() {
    setOutputPercent(m_outputSubscriber.get());
  }
}
