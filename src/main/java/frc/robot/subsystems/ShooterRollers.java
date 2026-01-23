// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SHOOTERMOTORS;
import frc.robot.Constants.SHOOTERMOTORS.ManualRPM;
import frc.team4201.lib.utils.CtreUtils;

public class ShooterRollers extends SubsystemBase {

  // TODO: Check how many motors we have later
  @Logged(name = "Flywheel Motor 1", importance = Importance.INFO)
  private final TalonFX m_motor1 = new TalonFX(CAN.kShooterRollerMotor1);

  // @Logged(name = "Flywheel Motor 2", importance = Importance.DEBUG)
  // private final TalonFX m_motor2 = new TalonFX(CAN.kShooterRollerMotor2);

  // @Logged(name = "Flywheel Motor 3", importance = Importance.DEBUG)
  // private final TalonFX m_motor3 = new TalonFX(CAN.kShooterRollerMotor3);

  // @Logged(name = "Flywheel Motor 4", importance = Importance.DEBUG)
  // private final TalonFX m_motor4 = new TalonFX(CAN.kShooterRollerMotor4);

  private NeutralModeValue m_neutralMode =
      NeutralModeValue.Coast; // Coast... because this is a flywheel. That coasts.
  private final MotionMagicVelocityTorqueCurrentFOC m_request =
      new MotionMagicVelocityTorqueCurrentFOC(0).withFeedForward(0.1);
  private final VoltageOut m_VoltageOut = new VoltageOut(0).withEnableFOC(true);
  private AngularVelocity m_rpmSetpoint = ManualRPM.IDLE.getRPM();

  private final FlywheelSim m_shooterMotorSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              SHOOTERMOTORS.gearbox, SHOOTERMOTORS.kInertia, SHOOTERMOTORS.gearRatio),
          SHOOTERMOTORS.gearbox);
  private final TalonFXSimState m_simState;

  private void sysIDLogMotors(SysIdRoutineLog log) {
    log.motor("motor1")
        .voltage(m_motor1.getMotorVoltage().refresh().getValue()) // Units: Volts
        .angularPosition(m_motor1.getPosition().refresh().getValue()) // Units: Rotations/Meters
        .angularVelocity(
            m_motor1.getVelocity().refresh().getValue()); // Units: Rotations per sec/Meters per sec
  }

  public ShooterRollers() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = SHOOTERMOTORS.kP;
    config.Slot0.kV = SHOOTERMOTORS.kV;
    config.Slot0.kS = SHOOTERMOTORS.kS;
    // config.Slot0.kA = SHOOTERMOTORS.kA;
    config.MotorOutput.NeutralMode = m_neutralMode;
    config.Feedback.SensorToMechanismRatio = SHOOTERMOTORS.gearRatio;
    config.MotorOutput.PeakForwardDutyCycle = SHOOTERMOTORS.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = SHOOTERMOTORS.peakReverseOutput;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicCruiseVelocity = SHOOTERMOTORS.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = SHOOTERMOTORS.motionMagicAcceleration;

    CtreUtils.configureTalonFx(m_motor1, config);
    // CtreUtils.configureTalonFx(m_motor2, config);



    m_simState = m_motor1.getSimState();

    // We only need the sim state of a single motor because all the motors are doing the same
    // thing... right???

    // m_motor2.setControl(new Follower(m_motor1.getDeviceID(), MotorAlignmentValue.Aligned));
    // m_motor3.setControl(new Follower(m_motor1.getDeviceID(), MotorAlignmentValue.Aligned));
    // m_motor4.setControl(new Follower(m_motor1.getDeviceID(), MotorAlignmentValue.Aligned));
    // TODO: Pls pls check if they all are actually aligned because it'd
    // be horrible if they weren't

  }

  public void changeNeutralMode(NeutralModeValue neutralmode) {
    m_neutralMode = neutralmode;
  }

  public void setManualRPMOutputFOC(AngularVelocity rpm) {
    m_rpmSetpoint = rpm;
    m_motor1.setControl(
        m_request.withVelocity(m_rpmSetpoint.abs(RotationsPerSecond)).withFeedForward(0.1));
  }

  public void setRPMOutputFOC(double rpm) {
    m_rpmSetpoint = RPM.of(rpm);
    m_motor1.setControl(
        m_request.withVelocity(m_rpmSetpoint.abs(RotationsPerSecond)).withFeedForward(0.1));
  }

  public void setVoltageOutputFOC(Voltage voltage) {
    m_motor1.setControl(m_VoltageOut.withOutput(voltage.abs(Volts)));
  }

  @Logged(name = "RPM Setpoint", importance = Logged.Importance.INFO)
  public double getRPMSetpoint() {
    return m_rpmSetpoint.in(RotationsPerSecond);
  }

  @Logged(name = "Motor Velocity in Rotations per Minute", importance = Logged.Importance.INFO)
  public double getMotorSpeedRotationsPerMinute() {
    return m_motor1.getVelocity().refresh().getValue().in(RotationsPerSecond) * 60;
  }

  public boolean[] isConnected() {
    return new boolean[] {
      m_motor1.isConnected()
    };
  }

  private SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(0.5), // Voltage change rate for quasistatic routine
              Volts.of(2), // Constant voltage value for dynamic routine
              null // Max time before automatically ending the routine
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

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_shooterMotorSim.setInputVoltage(m_simState.getMotorVoltage());

    m_shooterMotorSim.update(0.02);

    m_simState.setRawRotorPosition(
        Rotations.of(m_shooterMotorSim.getAngularVelocityRPM()).times(SHOOTERMOTORS.gearRatio));
    m_simState.setRotorVelocity(
        RPM.of(m_shooterMotorSim.getAngularVelocityRPM()).times(SHOOTERMOTORS.gearRatio));
  }
}
