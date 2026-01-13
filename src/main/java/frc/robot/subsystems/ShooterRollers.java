// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SHOOTERMOTORS;
import frc.team4201.lib.utils.CtreUtils;

public class ShooterRollers extends SubsystemBase {

  private final TalonFX[] m_motors = {
    new TalonFX(CAN.kShooterRollerMotor1),
    new TalonFX(CAN.kShooterRollerMotor2),
    new TalonFX(CAN.kShooterRollerMotor3),
    new TalonFX(CAN.kShooterRollerMotor4)
  }; // Replace these device ids after motors are set up

  private NeutralModeValue m_neutralMode =
      NeutralModeValue.Coast; // Coast... because this is a flywheel. That coasts.
  private final MotionMagicVelocityTorqueCurrentFOC m_request =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private double m_rpmSetpoint;

  private final DCMotorSim m_shooterMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              SHOOTERMOTORS.gearbox, SHOOTERMOTORS.kInertia, SHOOTERMOTORS.gearRatio),
          SHOOTERMOTORS.gearbox);
  private final TalonFXSimState m_simState;

  public ShooterRollers() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = SHOOTERMOTORS.kP;
    config.Slot0.kI = SHOOTERMOTORS.kI;
    config.Slot0.kD = SHOOTERMOTORS.kD;
    config.MotorOutput.NeutralMode = m_neutralMode;
    config.Feedback.SensorToMechanismRatio = SHOOTERMOTORS.gearRatio;
    config.MotorOutput.PeakForwardDutyCycle = SHOOTERMOTORS.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = SHOOTERMOTORS.peakReverseOutput;

    config.MotionMagic.MotionMagicCruiseVelocity = SHOOTERMOTORS.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = SHOOTERMOTORS.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = SHOOTERMOTORS.motionMagicJerk;

    for (int i = 0; i < m_motors.length; i++) {
      CtreUtils.configureTalonFx(m_motors[i], config);
    }

    m_simState =
        m_motors[0]
            .getSimState(); // We only need the sim state of a single motor because all the motors
    // are doing the same thing.

    for (int i = 1; i < m_motors.length; i++) {
      m_motors[i].setControl(
          new Follower(
              m_motors[0].getDeviceID(),
              MotorAlignmentValue
                  .Aligned)); // TODO: Pls pls check if they all are actually aligned because it'd
      // be horrible if they weren't
    }
  }

  @Logged(name = "Motor Velocity in Radians per Second", importance = Logged.Importance.INFO)
  public AngularVelocity getMotorSpeedRadians() {
    return m_motors[0].getVelocity().refresh().getValue();
  }

  @Logged(name = "Motor Voltage", importance = Logged.Importance.DEBUG)
  public Voltage getMotorVoltage() {
    return m_motors[0].getMotorVoltage().refresh().getValue();
  }

  public double getRPMSetpoint() {
    return m_rpmSetpoint;
  }

  public void setRPMOutputFOC(double rpm) {
    m_rpmSetpoint = rpm;
    var rps = rpm / 60;
    m_motors[0].setControl(m_request.withVelocity(rps).withFeedForward(0.1));
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_shooterMotorSim.setInputVoltage(m_simState.getMotorVoltage());

    m_shooterMotorSim.update(0.02);

    m_simState.setRawRotorPosition(
        Rotations.of(m_shooterMotorSim.getAngularPositionRotations())
            .times(SHOOTERMOTORS.gearRatio));
    m_simState.setRotorVelocity(
        RPM.of(m_shooterMotorSim.getAngularVelocityRPM()).times(SHOOTERMOTORS.gearRatio));
  }
}
