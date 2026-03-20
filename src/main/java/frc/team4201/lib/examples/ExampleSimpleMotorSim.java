// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4201.lib.examples;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.team4201.lib.simulation.TalonFXSim;
import frc.team4201.lib.utils.CtreUtils;

public class ExampleSimpleMotorSim extends SubsystemBase {

  // TODO: Check how many motors we have later
  @Logged(name = "Flywheel Motor 1", importance = Logged.Importance.INFO)
  private final TalonFX m_motor1 = new TalonFX(CAN.kShooterRollerMotor1, CAN.canivore);

  @Logged(name = "Flywheel Motor 2", importance = Logged.Importance.DEBUG)
  private final TalonFX m_motor2 = new TalonFX(CAN.kShooterRollerMotor2);

  @Logged(name = "Flywheel Motor 3", importance = Logged.Importance.DEBUG)
  private final TalonFX m_motor3 = new TalonFX(CAN.kShooterRollerMotor3);

  private final TalonFX[] mMotors = {m_motor1, m_motor2, m_motor3};

  private final NeutralModeValue mNeutralMode = NeutralModeValue.Coast;

  private final TorqueCurrentFOC mTorqueCurrentFOCRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC mVelocityRequest = new VelocityTorqueCurrentFOC(0);
  private static AngularVelocity mRpmSetpoint = RPM.zero();

  private final DCMotor mGearbox = DCMotor.getKrakenX60Foc(mMotors.length);
  private final double kInertia = 0.01;
  private final double kGearRatio = 1.0;
  private final double kVelocityErrorThreshold = 40.0;

  private final FlywheelSim m_shooterMotorSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(mGearbox, kInertia, kGearRatio), mGearbox);

  private final TalonFXSim mMotorSim =
      new TalonFXSim(m_shooterMotorSim, mMotors).withConversionFactor(kGearRatio);

  public ExampleSimpleMotorSim() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 1.0;
    config.MotorOutput.NeutralMode = mNeutralMode;
    config.Feedback.SensorToMechanismRatio = kGearRatio;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = false;

    for (int i = 0; i < mMotors.length; i++) {
      CtreUtils.configureDevice(mMotors[i], config);

      if (i != 0) {
        mMotors[i].setControl(new Follower(mMotors[0].getDeviceID(), MotorAlignmentValue.Aligned));
      }
    }
  }

  public void setOutput(Current output) {
    mMotors[0].setControl(mTorqueCurrentFOCRequest.withOutput(output));
  }

  public void setSetpoint(AngularVelocity rpm) {
    mRpmSetpoint = rpm;
  }

  @Logged(name = "Setpoint", importance = Logged.Importance.INFO)
  public AngularVelocity getSetpoint() {
    return mRpmSetpoint;
  }

  @Logged(name = "Velocity", importance = Logged.Importance.INFO)
  public AngularVelocity getVelocity() {
    return mMotors[0].getVelocity().getValue();
  }

  public boolean atSetpoint() {
    return getSetpointError().abs(RPM) <= kVelocityErrorThreshold;
  }

  public AngularVelocity getSetpointError() {
    return getSetpoint().minus(getVelocity());
  }

  @Override
  public void periodic() {
    mMotors[0].setControl(mVelocityRequest.withVelocity(getSetpoint()));
  }

  @Override
  public void simulationPeriodic() {
    mMotorSim.update();
  }
}
