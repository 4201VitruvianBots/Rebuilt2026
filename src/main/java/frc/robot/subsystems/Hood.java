// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.CAN;
import frc.robot.constants.FLYWHEEL;
import frc.robot.constants.FLYWHEEL.HOOD;
import frc.robot.constants.FLYWHEEL.HOOD.MANUAL_ANGLE;
import frc.team4201.lib.utils.CtreUtils;

public class Hood extends SubsystemBase {

  @Logged(name = "Hood Motor", importance = Importance.DEBUG)
  private final TalonFX m_motor =
      new TalonFX(
          CAN.kShooterHoodMotor, CAN.roboRIO); // Replace these device ids after motors are set up

  private final CANcoder m_cancoder =
      new CANcoder(
          CAN.kShooterHoodCANCoder,
          CAN.roboRIO); // Replace these device ids after motors are set up

  private DoublePublisher m_anglePublisher;

  private DoubleSubscriber m_angleSubscriber;

  private NeutralModeValue m_neutralMode =
      NeutralModeValue.Brake; // Brake... because this is a hood. That doesn't coast.
  private final MotionMagicVoltage m_request =
      new MotionMagicVoltage(Rotations.of(0.0)).withEnableFOC(false);
  private final VoltageOut m_VoltageOut = new VoltageOut(Volts.of(0)).withEnableFOC(false);

  private Angle m_hoodSetpoint = MANUAL_ANGLE.STOWED.getAngle();

  private final DCMotorSim m_shooterHoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(HOOD.gearbox, HOOD.kInertia, HOOD.gearRatio),
          HOOD.gearbox);

  private final TalonFXSimState m_simState = m_motor.getSimState();

  private final CANcoderSimState m_cancoderSimState = m_cancoder.getSimState();

  private void sysIDLogMotors(SysIdRoutineLog log) {
    log.motor("motor1")
        .voltage(m_motor.getMotorVoltage().refresh().getValue()) // Units: Volts
        .angularPosition(m_motor.getPosition().refresh().getValue()) // Units: Rotations/Meters
        .angularVelocity(
            m_motor.getVelocity().refresh().getValue()); // Units: Rotations per sec/Meters per sec
  }

  public Hood() {
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    if (RobotBase.isReal()) {
      encoderConfig.MagnetSensor.MagnetOffset = HOOD.kMagnetSensorOffset;
      encoderConfig.MagnetSensor.SensorDirection = HOOD.K_SENSOR_DIRECTION_VALUE;
      encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
      HOOD.kAbsoluteSensorDiscontinuityPoint;
    }
    CtreUtils.configureCANCoder(m_cancoder, encoderConfig);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = HOOD.kP;
    // config.Slot0.kA = HOOD.kA;
    // config.Slot0.kV = HOOD.kV;
    config.Slot0.kS = HOOD.kS;
    config.MotorOutput.NeutralMode = m_neutralMode;
    config.CurrentLimits.StatorCurrentLimit = HOOD.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.ClosedLoopGeneral.ContinuousWrap = false;
    if (RobotBase.isReal()) {
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    } else {
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = HOOD.rotorToSensorRatio;
    config.Feedback.FeedbackRemoteSensorID = m_cancoder.getDeviceID();
    config.Feedback.SensorToMechanismRatio = HOOD.gearRatio;

    config.MotionMagic.MotionMagicCruiseVelocity = HOOD.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = HOOD.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = HOOD.motionMagicJerk;


    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HOOD.maxAngle.in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HOOD.minAngle.in(Rotations);

    CtreUtils.configureTalonFx(m_motor, config);

    if (RobotBase.isSimulation()) m_cancoder.setPosition(MANUAL_ANGLE.STOWED.getAngle());
    m_motor.setPosition(
        getHoodAngle()
            .times(HOOD.gearRatio)
            .in(Rotations)); 
  }

  public void setAngle(Angle setpoint) {
    m_hoodSetpoint =
        Degrees.of(
            MathUtil.clamp(
                setpoint.in(Degrees), HOOD.minAngle.in(Degrees), HOOD.maxAngle.in(Degrees)));
    m_motor.setControl(m_request.withPosition(m_hoodSetpoint.in(Rotations)));
  }

  @Logged(name = "Hood Setpoint", importance = Importance.DEBUG)
  public Angle getDesiredAngle() {
    return m_hoodSetpoint;
  }

  public Voltage getHoodVoltage() {
    return m_motor.getMotorVoltage().refresh().getValue();
  }

  // TODO: Delete this after testing. Only for debugging purposes
  @Logged(name = "Hood Velocity", importance = Importance.DEBUG)
  public AngularVelocity getHoodVelocity() {
    return m_motor.getVelocity().refresh().getValue();
  }

  @Logged(name = "Hood Rotations", importance = Importance.DEBUG)
  public Angle getHoodAngle() {
    return m_cancoder
        .getPosition()
        .refresh()
        .getValue().div(HOOD.gearRatio); // Multiply by gear ratio to make hood angle more manageable
  }

  @Logged(name = "Hood Angle Degrees", importance = Importance.INFO)
  public double getHoodAngleDegrees() {
    return getHoodAngle().in(Degrees);
  }

  @Logged(name = "At Setpoint", importance = Logged.Importance.INFO)
  public boolean atSetpoint() {
    return (m_hoodSetpoint.in(Degrees) - getHoodAngleDegrees()) <= 1; // Works as good as always
  }

  public boolean[] isConnected() {
    return new boolean[] {m_motor.isConnected()};
  }

  public void setVoltageOutputFOC(Voltage voltage) {
    m_motor.setControl(m_VoltageOut.withOutput(voltage.in(Volts)));
  }

  public Command manualAgainstHubCommand() {
    return this.startEnd(() -> setAngle(MANUAL_ANGLE.HUB.getAngle()), () -> setAngle(Degrees.of(0.0)));
  }

  public Command manualAgainstTowerCommand() {
    return this.startEnd(() -> setAngle(MANUAL_ANGLE.TOWER.getAngle()), () -> setAngle(Degrees.of(0.0)));
  }

  public Command manualPassCommand() {
    return this.startEnd(() -> setAngle(MANUAL_ANGLE.PASSING.getAngle()), () -> setAngle(Degrees.of(0.0)));
  }

  @Override
  public void periodic() {
    // if (getHoodAngleDegrees() > HOOD.maxAngle.in(Degrees)) {
    //   m_motor.setControl(m_request.withPosition(HOOD.maxAngle.in(Rotations)));
    // } else {
    //   m_motor.setControl(m_request.withPosition(m_hoodSetpoint.in(Rotations)));
    // }
  }

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_shooterHoodSim.setInputVoltage(m_simState.getMotorVoltage());

    m_shooterHoodSim.update(0.02);

    m_simState.setRawRotorPosition(
        Rotations.of(m_shooterHoodSim.getAngularPositionRotations()).times(HOOD.gearRatio));
    m_simState.setRotorVelocity(
        RPM.of(m_shooterHoodSim.getAngularVelocityRPM()).times(HOOD.gearRatio));
    // Update the hoodEncoder simState
    m_cancoderSimState.setRawPosition(Rotations.of(m_shooterHoodSim.getAngularPositionRotations()));
    m_cancoderSimState.setVelocity(
        RadiansPerSecond.of(m_shooterHoodSim.getAngularVelocityRadPerSec()));
  }

  private SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(0.5), // Voltage change rate for quasistatic routine
              Volts.of(3), // Constant voltage value for dynamic routine
              null // Max time before automatically ending the routine, Defaults to 10 sec
              ),
          new SysIdRoutine.Mechanism(
              this::setVoltageOutputFOC, // Set voltage of mechanism
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

  // WARNING, ALL SYSID ROUTINES WILL NOT WORK BECAUSE IT'S USING VOLTAGE INSTEAD OF
  // TORQUECURRENTFOC

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void testInit() {
    var topic =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic("Hood Angle Setpoint");
    m_angleSubscriber = topic.subscribe(0.0);
    m_anglePublisher = topic.publish();
    m_anglePublisher.set(0.0);
  }

  public void testPeriodic() {
    setAngle(Degrees.of(m_angleSubscriber.get()));
  }
}
