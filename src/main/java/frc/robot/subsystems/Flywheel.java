// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Energy;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.CAN;
import frc.robot.constants.FLYWHEEL;
import frc.robot.constants.FLYWHEEL.MANUAL_RPM;
import frc.team4201.lib.utils.CtreUtils;

public class Flywheel extends SubsystemBase {

  // TODO: Check how many motors we have later
  @Logged(name = "Flywheel Motor 1", importance = Importance.INFO)
  private final TalonFX m_motor1 = new TalonFX(CAN.kShooterRollerMotor1, CAN.roboRIO);

  @Logged(name = "Flywheel Motor 2", importance = Importance.INFO)
  private final TalonFX m_motor2 = new TalonFX(CAN.kShooterRollerMotor2, CAN.roboRIO);

  @Logged(name = "Flywheel Motor 3", importance = Importance.INFO)
  private final TalonFX m_motor3 = new TalonFX(CAN.kShooterRollerMotor3, CAN.roboRIO);

  private NeutralModeValue m_neutralMode =
      NeutralModeValue.Coast; // Coast... because this is a flywheel. That coasts.

  private final VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0);
  private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
  private final VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private static AngularVelocity m_rpmSetpoint = MANUAL_RPM.IDLE.getRPM();
  private Energy m_totalEnergyUsed = Joules.of(0.0);
  private boolean m_isShooting = false;

  public final DoubleSubscriber m_rpmSubscriber;
  public final DoublePublisher m_rpmPublisher;

  private final FlywheelSim m_shooterMotorSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              FLYWHEEL.gearbox, FLYWHEEL.kInertia, FLYWHEEL.gearRatio),
          FLYWHEEL.gearbox);
  private final TalonFXSimState m_simState;

  private void sysIDLogFlywheelMotors(SysIdRoutineLog log) {
    log.motor("motor1")
        .voltage(m_motor1.getMotorVoltage().refresh().getValue()) // Units: Volts
        .angularPosition(m_motor1.getPosition().refresh().getValue()) // Units: Rotations/Meters
        .angularVelocity(
            m_motor1.getVelocity().refresh().getValue()); // Units: Rotations per sec/Meters per sec
  }

  public Flywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = FLYWHEEL.kP;
    // config.Slot0.kA = FLYWHEEL.kA;
    config.MotorOutput.NeutralMode = m_neutralMode;
    config.Feedback.SensorToMechanismRatio = FLYWHEEL.gearRatio;
    config.CurrentLimits.StatorCurrentLimit = FLYWHEEL.kStatorCurrentLimit;
    config.MotorOutput.PeakReverseDutyCycle = -0.1;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;

    CtreUtils.configureTalonFx(m_motor1, config);
    CtreUtils.configureTalonFx(m_motor2, config);
    CtreUtils.configureTalonFx(m_motor3, config);

    m_simState = m_motor1.getSimState();

    // We only need the sim state of a single motor

    m_motor2.setControl(new Follower(m_motor1.getDeviceID(), MotorAlignmentValue.Opposed));
    m_motor3.setControl(new Follower(m_motor1.getDeviceID(), MotorAlignmentValue.Opposed));

    var topic =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic("ShooterRPMSetpoint");
    m_rpmSubscriber = topic.subscribe(0.0);
    m_rpmPublisher = topic.publish();
  }

  public void changeNeutralMode(NeutralModeValue neutralmode) {
    m_neutralMode = neutralmode;
  }

  public void setRPMOutput(AngularVelocity rpm) {
    m_rpmSetpoint = rpm;
  }

  @Logged(name = "Is Shooting?", importance = Logged.Importance.INFO)
  public boolean getIsShooting() {
    return m_isShooting;
  }

  public void setIsShooting(boolean isShooting){
    m_isShooting = isShooting;
  }

  public void setVoltageOutput(Voltage voltage) {
    m_motor1.setControl(m_voltageOut.withOutput(voltage.abs(Volts)));
    m_rpmSetpoint = RPM.of(0);
  }

  // public void setTorqueCurrentOutputFOC(Voltage voltage) {
  //   m_motor1.setControl(m_torqueCurrentFOC.withOutput(voltage.abs(Volts)));
  // }

  @Logged(name = "RPM Setpoint", importance = Logged.Importance.INFO)
  public double getRPMSetpoint() {
    return m_rpmSetpoint.in(RPM);
  }

  @Logged(name = "Motor Velocity RPM", importance = Logged.Importance.INFO)
  public double getMotorSpeedRPM() {
    return m_motor1.getVelocity().refresh().getValue().in(RPM);
  }

  public boolean[] isConnected() {
    return new boolean[] {m_motor1.isConnected()};
  }

  public Current getSupplyCurrent() {
    return m_motor1.getSupplyCurrent().refresh().getValue();
  }

  public Power getPowerDraw() {
    double power = (getSupplyCurrent().times(RobotController.getBatteryVoltage())).in(Amp);
    return Watts.of(power);
  }

  public void updateEnergyUsed() {
    double newEnergy = (getPowerDraw().in(Watts) * 0.02);
    m_totalEnergyUsed = m_totalEnergyUsed.plus(Joules.of(newEnergy));
  }

  @Logged(name = "Total Energy Used by Flywheel", importance = Importance.INFO)
  public Energy getEnergyUsed() {
    return m_totalEnergyUsed;
  }

  private SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(0.5), // Voltage change rate for quasistatic routine
              Volts.of(2), // Constant voltage value for dynamic routine
              null // Max time before automatically ending the routine
              ),
          new SysIdRoutine.Mechanism(
              this::setVoltageOutput, // Set voltage of mechanism
              this::sysIDLogFlywheelMotors,
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

  public Command manualAgainstHubCommand() {
    return this.startEnd(
        () -> setRPMOutput(MANUAL_RPM.HUB.getRPM()), () -> setVoltageOutput(Volts.of(0.0)));
  }

  public Command manualAgainstTowerCommand() {
    return this.startEnd(
        () -> setRPMOutput(MANUAL_RPM.TOWER.getRPM()),
        () -> setVoltageOutput(Volts.of(0.0))); // Unverified
  }

  public Command manualPassCommand() {
    return this.startEnd(
        () -> setRPMOutput(MANUAL_RPM.PASSING.getRPM()), () -> setVoltageOutput(Volts.of(0.0)));
  }

  public void testInit() {
    m_rpmPublisher.set(0.0);
  }

  public void testPeriodic() {
    m_rpmSetpoint = RPM.of(m_rpmSubscriber.get());
  }

  public boolean isAtRPMsetpoint() {
    if (DriverStation.isAutonomous()) {
      return getAbsoluteRPMerror() <= FLYWHEEL.kVelocityErrorThresholdAuto;
    } else {
      return getAbsoluteRPMerror() <= FLYWHEEL.kVelocityErrorThresholdTeleop;
    }
  }

  public double getRPMerror() {
    return getRPMSetpoint() - getMotorSpeedRPM();
  }

  @Logged(name = "RPM error", importance = Importance.DEBUG)
  public double getAbsoluteRPMerror() {
    return Math.abs(getRPMerror());
  }

  // public boolean getShouldRev(){
  //   boolean shiftEnding = HubTracker.timeRemainingInCurrentShift().isPresent() && HubTracker.timeRemainingInCurrentShift().get().abs(Seconds) < 3;
  //   boolean shouldRev = (HubTracker.isActive() || shiftEnding) && m_vision != null && !m_vision.isInOpposingAllianceSector();
  //   return shouldRev;
  // }

  @Override
  public void periodic() {
    m_motor1.setControl(m_request.withVelocity(m_rpmSetpoint.abs(RotationsPerSecond)));
    updateEnergyUsed();
  }

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_shooterMotorSim.setInputVoltage(m_simState.getMotorVoltage());

    m_shooterMotorSim.update(0.02);

    m_simState.setRawRotorPosition(
        Rotations.of(m_shooterMotorSim.getAngularVelocityRPM()).times(FLYWHEEL.gearRatio));
    m_simState.setRotorVelocity(
        RPM.of(m_shooterMotorSim.getAngularVelocityRPM()).times(FLYWHEEL.gearRatio));
  }
}
