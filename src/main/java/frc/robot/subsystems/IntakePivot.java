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

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.INTAKE.PIVOT;
import frc.robot.constants.INTAKE.PIVOT.PIVOT_SETPOINT;
import frc.team4201.lib.utils.CtreUtils;

public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
  @Logged(name = "Intake Pivot Motor", importance = Importance.INFO)
  private final TalonFX m_motor = new TalonFX(CAN.kIntakePivotMotor, CAN.roboRIO);

  private final CANcoder m_canCoder = new CANcoder(CAN.kPivotEncoder, CAN.roboRIO);

  private DoubleSubscriber m_angleSubscriber;
  private DoublePublisher m_anglePublisher;

  private final MotionMagicVoltage m_request = new MotionMagicVoltage(Rotations.of(0.0));

  private static Angle m_desiredAngle = PIVOT_SETPOINT.INTAKING.getAngle();

  private final TalonFXSimState m_motorSimState = m_motor.getSimState();
  private final CANcoderSimState m_cancoderSimState = m_canCoder.getSimState();

  private boolean m_manualOverride = false;

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
      encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
          PIVOT.kAbsoluteSensorDiscontinuityPoint;
    }

    CtreUtils.configureCANCoder(m_canCoder, encoderConfig);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = PIVOT.kP;
    config.Slot0.kD = PIVOT.kD;
    config.Slot0.kG = PIVOT.kG;
    // config.Slot0.kA = PIVOT.kA;
    // config.Slot0.kV = PIVOT.kV;
    // config.Slot0.kS = PIVOT.kS;
    config.Slot0.GravityType = PIVOT.K_GRAVITY_TYPE_VALUE;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.FeedbackRemoteSensorID = m_canCoder.getDeviceID();
    config.CurrentLimits.StatorCurrentLimit = PIVOT.kStatorCurrentLimit;
    config.Feedback.SensorToMechanismRatio = PIVOT.SensorToMechanismRatio;
    config.Feedback.RotorToSensorRatio = PIVOT.gearRatio;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    // config.ClosedLoopGeneral.ContinuousWrap = false;

    // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PIVOT.maxAngle.in(Rotations);
    // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PIVOT.minAngle.in(Rotations);

    config.MotionMagic.MotionMagicCruiseVelocity = PIVOT.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = PIVOT.motionMagicAcceleration;

    CtreUtils.configureTalonFx(m_motor, config);

    if (RobotBase.isSimulation()) {
      m_motor.setPosition(PIVOT.startingAngle.in(Rotations));
      m_canCoder.setPosition(PIVOT.startingAngle.in(Rotations));
    }

    m_motor.setPosition(getAngle().times(PIVOT.SensorToMechanismRatio).in(Rotations));
  }

  public void setAngle(Angle angle) {
    m_desiredAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees), PIVOT.minAngle.in(Degrees), PIVOT.maxAngle.in(Degrees)));
  }

  @Logged(name = "Pivot Setpoint", importance = Importance.INFO)
  public double getDesiredAngle() {
    return m_desiredAngle.in(Degrees);
  }

  @Logged(name = "Pivot Angle Radians", importance = Importance.DEBUG)
  public Angle getAngle() {
    return m_canCoder.getPosition().refresh().getValue().div(PIVOT.SensorToMechanismRatio);
  }

  @Logged(name = "Pivot Angle Degrees", importance = Importance.INFO)
  public double getAngleDegrees() {
    return getAngle().in(Degrees);
  }

  @Logged(name = "At Setpoint", importance = Logged.Importance.DEBUG)
  public boolean atSetpoint() {
    return m_desiredAngle.minus(getAngle()).abs(Degrees) <= 1; // Works as good as always
  }

  public boolean isConnected() {
    return m_motor.isConnected();
  }

  // placeholder, idea (in the future) is to find
  // way to track previous setpoint and use that for jostling (like if the previous was stowed then
  // not be able to jostle on accident)
  // public Boolean prevSetpointIsIntaking() {
  //  return m_desiredAngle
  // }

  @NotLogged
  public Command command(PIVOT_SETPOINT setpoint) {
    return this.runOnce(() -> setAngle(setpoint.getAngle()));
  }

  @NotLogged
  public Command manualOpenLoopOverride(DoubleSupplier speed) {
    return new InstantCommand(() -> m_manualOverride = true)
    .andThen(this.runEnd(() -> m_motor.set(speed.getAsDouble()), () -> {
      m_manualOverride = false; 
      m_desiredAngle = getAngle();
    }));
  }

  // Commented out because it was old, am replacing with seperate command file
  // @NotLogged
  // public Command jostle() {
  //   // return new RepeatCommand(
  //   //     this.startRun(
  //   //             () -> {

  //   //             },
  //   //             () -> {
  //   //               setAngle(PIVOT_SETPOINT.INTAKING.getAngle());
  //   //             })
  //   //         .withTimeout(0.15)
  //   //         .andThen(new WaitCommand(0.1)));
  // }

  @Override
  public void periodic() {
    if (!m_manualOverride) {
      m_motor.setControl(m_request.withPosition(m_desiredAngle.in(Rotations)));
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

  public void testInit() {
    var topic =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic("Intake Angle Setpoint");
    m_angleSubscriber = topic.subscribe(0.0);
    m_anglePublisher = topic.publish();
    m_anglePublisher.set(PIVOT_SETPOINT.INTAKING.getAngle().abs(Degrees));
  }

  public void testPeriodic() {
    setAngle(Degrees.of(m_angleSubscriber.get()));
  }
}
