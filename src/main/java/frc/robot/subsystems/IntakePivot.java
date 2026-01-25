// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.INTAKEMOTORS.PIVOT;
import frc.robot.Constants.INTAKEMOTORS.PIVOT.PIVOT_SETPOINT;
import frc.robot.Constants.SHOOTERHOOD;
import frc.team4201.lib.utils.CtreUtils;

public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
  @Logged(name = "Intake Pivot Motor", importance = Importance.INFO)
  private final TalonFX m_motor = new TalonFX(CAN.kIntakePivotMotor);

  private final CANcoder m_canCoder = new CANcoder(CAN.kPivotEncoder);

  private final StatusSignal<Angle> m_positionSignal = m_motor.getPosition().clone();

  private final MotionMagicTorqueCurrentFOC m_request =
      new MotionMagicTorqueCurrentFOC(Rotations.of(0.0));

  private static Angle m_desiredAngle = PIVOT_SETPOINT.STOWED.getAngle();

  private final TalonFXSimState m_motorSimState = m_motor.getSimState();
  private final CANcoderSimState m_cancoderSimState = m_canCoder.getSimState();

  // Simulation Code
  private final SingleJointedArmSim m_pivotSim =
      new SingleJointedArmSim(
          PIVOT.gearbox,
          PIVOT.gearRatio,
          SingleJointedArmSim.estimateMOI(PIVOT.baseLength.in(Meters), PIVOT.mass.in(Kilograms)),
          PIVOT.baseLength.in(Meters),
          PIVOT.minAngle.in(Radians),
          PIVOT.maxAngle.in(Radians),
          true,
          PIVOT.startingAngle.in(Radians));

  public IntakePivot() {
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    if (RobotBase.isReal()) {
      encoderConfig.MagnetSensor.MagnetOffset = PIVOT.encoderOffset;
      encoderConfig.MagnetSensor.SensorDirection = PIVOT.encoderDirection;
    }

    CtreUtils.configureCANCoder(m_canCoder, encoderConfig);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = PIVOT.kP;
    config.Slot0.kD = PIVOT.kD;
    // config.Slot0.kA = PIVOT.kA;
    // config.Slot0.kV = PIVOT.kV;
    // config.Slot0.kS = PIVOT.kS;
    // config.Slot0.GravityType = PIVOT.K_GRAVITY_TYPE_VALUE;

    config.Feedback.SensorToMechanismRatio = PIVOT.gearRatio;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = m_canCoder.getDeviceID();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // config.CurrentLimits.StatorCurrentLimit = 30;
    // config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.ClosedLoopGeneral.ContinuousWrap = false;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PIVOT.maxAngle.in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PIVOT.minAngle.in(Rotations);

    config.MotionMagic.MotionMagicCruiseVelocity = PIVOT.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = PIVOT.motionMagicAcceleration;

    CtreUtils.configureTalonFx(m_motor, config);

    if (RobotBase.isSimulation()) {
      m_motor.setPosition(PIVOT.startingAngle.in(Rotations));
      m_canCoder.setPosition(PIVOT.startingAngle.in(Rotations));
    }
  }

  public void setAngle(Angle angle) {
    m_desiredAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees), PIVOT.minAngle.in(Degrees), PIVOT.maxAngle.in(Degrees)));
    m_motor.setControl(m_request.withPosition(m_desiredAngle.in(Rotations)));
    SmartDashboard.putString("Pivot Control Request", m_request.withPosition(m_desiredAngle.in(Rotations)).toString());
  }

  @Logged(name = "Pivot Setpoint", importance = Importance.INFO)
  public double getDesiredAngle() {
    return m_desiredAngle.in(Degrees);
  }

  @Logged(name = "Pivot Angle Radians", importance = Importance.DEBUG)
  public Angle getAngle() {
    return m_canCoder.getAbsolutePosition().refresh().getValue();
  }

  @Logged(name = "Pivot Angle Degrees", importance = Importance.INFO)
  public double getAngleDegrees() {
    return getAngle().in(Degrees);
  }

  @Logged(name = "At Setpoint", importance = Logged.Importance.INFO)
  public boolean atSetpoint() {
    return m_desiredAngle.minus(getAngle()).abs(Degrees) <= 1; // Works as good as always
  }

  public boolean isConnected() {
    return m_motor.isConnected();
  }

  @Override
  public void periodic() {
    if (getAngleDegrees() > PIVOT.maxAngle.in(Degrees)) {
      m_motor.setControl(m_request.withPosition(PIVOT.maxAngle.in(Rotations)));
    }
  }

  @Override
  public void simulationPeriodic() {
    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_pivotSim.setInputVoltage(m_motorSimState.getMotorVoltage());

    m_pivotSim.update(0.02);

    m_motorSimState.setRawRotorPosition(Radians.of(m_pivotSim.getAngleRads()));
    m_motorSimState.setRotorVelocity(RadiansPerSecond.of(m_pivotSim.getVelocityRadPerSec()));

    
    // Update the pivotEncoder simState
    m_cancoderSimState.setRawPosition(Radians.of(m_pivotSim.getAngleRads()));
    m_cancoderSimState.setVelocity(RadiansPerSecond.of(m_pivotSim.getVelocityRadPerSec()));

  }
 
}
