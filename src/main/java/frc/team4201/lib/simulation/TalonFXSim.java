package frc.team4201.lib.simulation;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.*;
import java.util.HashMap;
import java.util.Map;

public class TalonFXSim {

  enum MODEL_TYPE {
    SIMPLE_MOTOR,
    FLYWHEEL,
    ELEVATOR,
    ARM,
  }

  private final Map<TalonFX, TalonFXSimState> mMotors = new HashMap<>();
  private final Map<TalonFX, CANcoderSimState> mCanCoderSimStates = new HashMap<>();
  private final Map<TalonFX, Double> mCanCoderRatios = new HashMap<>();
  private final LinearSystemSim<?, ?, ?> mModel;
  private final MODEL_TYPE mModelType;
  private Per<AngleUnit, DistanceUnit> mConversionFactor;
  private Dimensionless mConversionFactor2 = Value.of(1);
  private Angle angle;
  private Time mLastTimestamp = Seconds.zero();

  /** */
  public TalonFXSim(LinearSystemSim<?, ?, ?> model, TalonFX... motors) {
    mModel = model;

    if (mModel instanceof DCMotorSim) {
      mModelType = MODEL_TYPE.SIMPLE_MOTOR;
      angle = Radian.zero();
    } else if (mModel instanceof FlywheelSim) {
      mModelType = MODEL_TYPE.FLYWHEEL;
      angle = Radian.zero();
    } else if (mModel instanceof ElevatorSim) mModelType = MODEL_TYPE.ELEVATOR;
    else if (mModel instanceof SingleJointedArmSim) mModelType = MODEL_TYPE.ARM;
    else throw new RuntimeException("[ERROR] Unsupported sim model type!");

    for (var motor : motors) {
      mMotors.put(motor, motor.getSimState());

      var config = new TalonFXConfiguration();
      motor.getConfigurator().refresh(config);
      // TODO: Check if re-declaring a CANcoder will break code
      if (config.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RemoteCANcoder) {
        var canCoder = new CANcoder(config.Feedback.FeedbackRemoteSensorID);
        mCanCoderSimStates.put(motor, canCoder.getSimState());

        // TODO: This check can fail
        if (config.Feedback.RotorToSensorRatio != 1) {
          mCanCoderRatios.put(motor, config.Feedback.RotorToSensorRatio);
        } else if (config.Feedback.SensorToMechanismRatio != 1) {
          mCanCoderRatios.put(motor, config.Feedback.SensorToMechanismRatio);
        } else throw new RuntimeException("[ERROR] CanCoder ratio does not appear to be set!");
      }
    }
  }

  public TalonFXSim withConversionFactor(Per<AngleUnit, DistanceUnit> conversionFactor) {
    mConversionFactor = conversionFactor;
    return this;
  }

  public TalonFXSim withConversionFactor(double conversionFactor) {
    return withConversionFactor(Value.of(conversionFactor));
  }

  public TalonFXSim withConversionFactor(Dimensionless conversionFactor) {
    mConversionFactor2 = conversionFactor;
    return this;
  }

  public void update() {
    // Set motor inputs
    for (var simState : mMotors.values()) {
      simState.setSupplyVoltage(RobotController.getBatteryVoltage());
      mModel.setInput(simState.getMotorVoltage());
    }

    // Update the model
    mModel.update(0.02);
    var currentTimestamp = Seconds.of(Timer.getFPGATimestamp());

    // Feed in the model values to the motors
    mMotors.forEach(
        (motor, simState) -> {
          switch (mModelType) {
            case ARM -> {
              // Radians
              simState.setRawRotorPosition(
                  Radians.of(mModel.getOutput(0)).times(mConversionFactor2));
              simState.setRotorVelocity(
                  RadiansPerSecond.of(mModel.getOutput(1)).times(mConversionFactor2));
            }
            case ELEVATOR -> {
              // Meters
              var position = Meters.of(mModel.getOutput(0));
              var velocity = MetersPerSecond.of(mModel.getOutput(1));
              //                m_simState.setRawRotorPosition(position.times(m_conversionFactor));
              //                m_simState.setRotorVelocity(velocity.times(m_conversionFactor));
            }
            case FLYWHEEL -> {
              var flywheel = (FlywheelSim) mModel;
              // Radians
              angle.plus(
                  flywheel.getAngularVelocity().times(currentTimestamp.minus(mLastTimestamp)));
              simState.setRawRotorPosition(angle);
              simState.setRotorVelocity(flywheel.getAngularVelocity());
              simState.setRotorAcceleration(flywheel.getAngularAcceleration());

              // Update the CANcoder if it is set
              if (mCanCoderSimStates.containsKey(motor)) {
                var canCoderSimState = mCanCoderSimStates.get(motor);
                var canCoderRatio = mCanCoderRatios.get(motor);
                canCoderSimState.setRawPosition(angle.times(canCoderRatio));
                canCoderSimState.setVelocity(flywheel.getAngularVelocity().times(canCoderRatio));
              }
            }
            case SIMPLE_MOTOR -> {
              var flywheel = (DCMotorSim) mModel;
              // Radians
              angle.plus(
                  flywheel.getAngularVelocity().times(currentTimestamp.minus(mLastTimestamp)));
              simState.setRawRotorPosition(angle);
              simState.setRotorVelocity(flywheel.getAngularVelocity());
              simState.setRotorAcceleration(flywheel.getAngularAcceleration());
              if (mCanCoderSimStates.containsKey(motor)) {
                var canCoderSimState = mCanCoderSimStates.get(motor);
                var canCoderRatio = mCanCoderRatios.get(motor);
                canCoderSimState.setRawPosition(angle.times(canCoderRatio));
                canCoderSimState.setVelocity(flywheel.getAngularVelocity().times(canCoderRatio));
              }
            }
          }
        });
    mLastTimestamp = currentTimestamp;
  }
}
