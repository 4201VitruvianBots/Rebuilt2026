// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.CAN;
import frc.robot.constants.FLYWHEEL;
import frc.robot.constants.FLYWHEEL.MANUAL_RPM;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredDevice;
import frc.team4201.lib.hardwareMonitor.annotations.MonitoredSubsystem;
import frc.team4201.lib.simulation.TalonFXSim;
import frc.team4201.lib.utils.CtreUtils;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.Map;

public class Flywheel extends SubsystemBase implements MonitoredSubsystem {

  // TODO: Check how many motors we have later
  @MonitoredDevice(
      name = "Flywheel Motor 1",
      type = MonitoredDevice.DEVICE_TYPE.PRIMARY,
      canbus = "rio")
  @Logged(name = "Flywheel Motor 1", importance = Importance.INFO)
  private final TalonFX m_motor1 = new TalonFX(CAN.kShooterRollerMotor1, CAN.roboRIO);

  @MonitoredDevice(
      name = "Flywheel Motor 2",
      type = MonitoredDevice.DEVICE_TYPE.SECONDARY,
      canbus = "rio")
  @Logged(name = "Flywheel Motor 2", importance = Importance.DEBUG)
  private final TalonFX m_motor2 = new TalonFX(CAN.kShooterRollerMotor2, CAN.roboRIO);

  @MonitoredDevice(
      name = "Flywheel Motor 3",
      type = MonitoredDevice.DEVICE_TYPE.SECONDARY,
      canbus = "rio")
  @Logged(name = "Flywheel Motor 3", importance = Importance.DEBUG)
  private final TalonFX m_motor3 = new TalonFX(CAN.kShooterRollerMotor3, CAN.roboRIO);

  private final TalonFX[] m_Motors = {m_motor1, m_motor2, m_motor3};
  private final LinkedHashMap<TalonFX, LinkedList<BaseStatusSignal>> m_motorSignals =
      new LinkedHashMap<>(
          Map.ofEntries(
              Map.entry(m_motor1, new LinkedList<>()),
              Map.entry(m_motor2, new LinkedList<>()),
              Map.entry(m_motor3, new LinkedList<>())));

  private NeutralModeValue m_neutralMode =
      NeutralModeValue.Coast; // Coast... because this is a flywheel. That coasts.

  private final VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0);
  private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
  private final TorqueCurrentFOC m_torqueCurrentFOC = new TorqueCurrentFOC(0.0);
  private static AngularVelocity m_rpmSetpoint = MANUAL_RPM.IDLE.getRPM();

  public final DoubleSubscriber m_rpmSubscriber;
  public final DoublePublisher m_rpmPublisher;

  private final FlywheelSim m_shooterMotorSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              FLYWHEEL.gearbox, FLYWHEEL.kInertia, FLYWHEEL.gearRatio),
          FLYWHEEL.gearbox);
  private final TalonFXSim m_motorSim =
      new TalonFXSim(m_shooterMotorSim, m_motor1, m_motor2, m_motor3)
          .withConversionFactor(FLYWHEEL.gearRatio);

  private void sysIDLogMotors(SysIdRoutineLog log) {
    log.motor("motor1")
        .voltage(m_motor1.getMotorVoltage().getValue()) // Units: Volts
        .angularPosition(m_motor1.getPosition().getValue()) // Units: Rotations/Meters
        .angularVelocity(
            m_motor1.getVelocity().getValue()); // Units: Rotations per sec/Meters per sec
  }

  public Flywheel() {
    // TODO: Check if they all are  aligned
    var topic =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic("ShooterRPMSetpoint");
    m_rpmSubscriber = topic.subscribe(0.0);
    m_rpmPublisher = topic.publish();
  }

  @Override
  public void initDevices() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = FLYWHEEL.kP;
    config.Slot0.kD = FLYWHEEL.kD;
    config.Slot0.kV = FLYWHEEL.kV;
    config.Slot0.kS = FLYWHEEL.kS;
    // config.Slot0.kA = FLYWHEEL.kA;
    config.MotorOutput.NeutralMode = m_neutralMode;
    config.Feedback.SensorToMechanismRatio = FLYWHEEL.gearRatio;
    config.CurrentLimits.StatorCurrentLimit = FLYWHEEL.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = false;

    for (var m : m_Motors) {
      CtreUtils.configureDevice(m, config);
    }

    m_motor2.setControl(new Follower(m_motor1.getDeviceID(), MotorAlignmentValue.Aligned));
    m_motor3.setControl(new Follower(m_motor1.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public void changeNeutralMode(NeutralModeValue neutralMode) {
    m_neutralMode = neutralMode;
  }

  public void setRPMOutputFOC(AngularVelocity rpm) {
    m_rpmSetpoint = rpm;
  }

  public void setTorqueCurrentOutputFOC(Voltage voltage) {
    m_motor1.setControl(m_torqueCurrentFOC.withOutput(voltage.abs(Volts)));
  }

  @Logged(name = "RPM Setpoint", importance = Logged.Importance.INFO)
  public double getRPMSetpoint() {
    return m_rpmSetpoint.in(RPM);
  }

  @Logged(name = "Motor Velocity RPM", importance = Logged.Importance.INFO)
  public double getMotorSpeedRPM() {
    return m_motor1.getVelocity().getValue().in(RPM);
  }

  public boolean[] isConnected() {
    return new boolean[] {m_motor1.isConnected()};
  }

  private SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(0.5), // Voltage change rate for quasistatic routine
              Volts.of(2), // Constant voltage value for dynamic routine
              null // Max time before automatically ending the routine
              ),
          new SysIdRoutine.Mechanism(
              this::setTorqueCurrentOutputFOC, // Set voltage of mechanism
              this::sysIDLogMotors,
              this));

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Command manualCommand() {
    return this.runEnd(
        () -> setRPMOutputFOC(RPM.of(getRPMSetpoint())),
        () -> setTorqueCurrentOutputFOC(Volts.zero()));
  }

  public void testInit() {
    m_rpmPublisher.set(0.0);
  }

  public void testPeriodic() {
    m_rpmSetpoint = RPM.of(m_rpmSubscriber.get());
  }

  public boolean isAtRPMsetpoint() {
    return getAbsoluteRPMerror() <= FLYWHEEL.kVelocityErrorThreshold;
  }

  public double getRPMerror() {
    return getRPMSetpoint() - getMotorSpeedRPM();
  }

  @Logged(name = "RPM error", importance = Importance.DEBUG)
  public double getAbsoluteRPMerror() {
    return Math.abs(getRPMerror());
  }

  @Override
  public void periodic() {
    if (!isAtRPMsetpoint()) {
      m_motor1.setControl(m_dutyCycleOut.withOutput(Math.signum(getRPMerror())));
    } else {
      m_motor1.setControl(m_request.withVelocity(m_rpmSetpoint.abs(RotationsPerSecond)));
    }
  }

  @Override
  public void simulationPeriodic() {
    m_motorSim.update();
  }
}
