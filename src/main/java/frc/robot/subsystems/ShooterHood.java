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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
import frc.robot.Constants.CAN;
import frc.robot.Constants.INTAKEMOTORS.PIVOT;
import frc.robot.Constants.SHOOTERHOOD;
import frc.robot.Constants.SHOOTERHOOD.HoodAngle;
import frc.team4201.lib.utils.CtreUtils;

public class ShooterHood extends SubsystemBase {

  @Logged(name = "Hood Motor", importance = Importance.DEBUG)
  private final TalonFX m_motor =
      new TalonFX(CAN.kShooterHoodMotor); // Replace these device ids after motors are set up

  private final CANcoder m_cancoder =
      new CANcoder(CAN.kShooterHoodCANCoder); // Replace these device ids after motors are set up

  private NeutralModeValue m_neutralMode =
      NeutralModeValue.Brake; // Brake... because this is a hood. That doesn't coast.
  private final MotionMagicVoltage m_request =
      new MotionMagicVoltage(Rotations.of(0.0)).withEnableFOC(true);
  private final VoltageOut m_VoltageOut = new VoltageOut(Volts.of(0)).withEnableFOC(true);

  private Angle m_hoodSetpoint = HoodAngle.NOTHING.getAngle();

  private final DCMotorSim m_shooterHoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              SHOOTERHOOD.gearbox, SHOOTERHOOD.kInertia, SHOOTERHOOD.gearRatio),
          SHOOTERHOOD.gearbox);

  private final TalonFXSimState m_simState = m_motor.getSimState();
  
  private final CANcoderSimState m_cancoderSimState = m_cancoder.getSimState();

  private void sysIDLogMotors(SysIdRoutineLog log) {
    log.motor("motor1")
        .voltage(m_motor.getMotorVoltage().refresh().getValue()) // Units: Volts
        .angularPosition(m_motor.getPosition().refresh().getValue()) // Units: Rotations/Meters
        .angularVelocity(
            m_motor.getVelocity().refresh().getValue()); // Units: Rotations per sec/Meters per sec
  }

  public ShooterHood() {
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    if (RobotBase.isReal()) {
      encoderConfig.MagnetSensor.MagnetOffset = PIVOT.encoderOffset;
      encoderConfig.MagnetSensor.SensorDirection = PIVOT.encoderDirection;
    }

    CtreUtils.configureCANCoder(m_cancoder, encoderConfig);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = SHOOTERHOOD.kP;
    config.Slot0.kD = SHOOTERHOOD.kD;
    // config.Slot0.kA = SHOOTERHOOD.kA;
    // config.Slot0.kV = SHOOTERHOOD.kV;
    // config.Slot0.kS = SHOOTERHOOD.kS;
    config.MotorOutput.NeutralMode = m_neutralMode;
    config.MotorOutput.PeakForwardDutyCycle = SHOOTERHOOD.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = SHOOTERHOOD.peakReverseOutput;
    config.CurrentLimits.StatorCurrentLimit = 30;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.ClosedLoopGeneral.ContinuousWrap = false;

    config.Feedback.SensorToMechanismRatio = SHOOTERHOOD.gearRatio;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = m_cancoder.getDeviceID();

    config.MotionMagic.MotionMagicCruiseVelocity = SHOOTERHOOD.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = SHOOTERHOOD.motionMagicAcceleration;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SHOOTERHOOD.maxAngle.in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SHOOTERHOOD.minAngle.in(Rotations);

    if (RobotBase.isSimulation()) m_cancoder.setPosition(HoodAngle.NOTHING.getAngle());
    m_motor.setPosition(getHoodRotations().in(Rotations));

    CtreUtils.configureTalonFx(m_motor, config);
  }

  public void setAngle(Angle setpoint) {
    m_hoodSetpoint =
        Degrees.of(
            MathUtil.clamp(
                setpoint.in(Degrees),
                SHOOTERHOOD.minAngle.in(Degrees),
                SHOOTERHOOD.maxAngle.in(Degrees)));
    m_motor.setControl(m_request.withPosition(m_hoodSetpoint.in(Rotations)));
  }

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
  public Angle getHoodRotations() {
    return m_cancoder.getAbsolutePosition().refresh().getValue();
  }

  @Logged(name = "Hood Angle", importance = Importance.INFO)
  public double getHoodAngle() {
    return getHoodRotations().in(Degrees);
  }

  @Logged(name = "At Setpoint", importance = Logged.Importance.INFO)
  public boolean atSetpoint() {
    return m_hoodSetpoint.minus(getHoodRotations()).abs(Degrees) <= 1; // Works as good as always
  }

  public boolean[] isConnected() {
    return new boolean[] {m_motor.isConnected()};
  }

  public void setVoltageOutputFOC(Voltage voltage) {
    m_motor.setControl(m_VoltageOut.withOutput(voltage.in(Volts)));
  }

  @Override
  public void periodic() {
    if (getHoodAngle() > SHOOTERHOOD.maxAngle.in(Degrees)) {
      m_motor.setControl(m_request.withPosition(SHOOTERHOOD.maxAngle.in(Rotations)));
    }
  }

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_shooterHoodSim.setInputVoltage(m_simState.getMotorVoltage());

    m_shooterHoodSim.update(0.02);

    m_simState.setRawRotorPosition(Rotations.of(m_shooterHoodSim.getAngularPositionRotations()));
    m_simState.setRotorVelocity(RPM.of(m_shooterHoodSim.getAngularVelocityRPM()));
    // Update the pivotEncoder simState
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

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
