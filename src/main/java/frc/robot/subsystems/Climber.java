package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.CLIMBER.HOLDING_ROBOT;
import frc.robot.constants.CLIMBER.NOT_HOLDING_ROBOT;
import frc.team4201.lib.utils.CtreUtils;

public class Climber extends SubsystemBase {
  // Creates a new motor object.
  @Logged(name = "Climber Motor", importance = Importance.DEBUG)
  private final TalonFX m_climberMotor = new TalonFX(CAN.kClimberMotor);

  private final DynamicMotionMagicVoltage m_request =
      new DynamicMotionMagicVoltage(
              0.0,
              CLIMBER.NOT_HOLDING_ROBOT.motionMagicCruiseVelocity,
              CLIMBER.NOT_HOLDING_ROBOT.motionMagicAcceleration)
          .withEnableFOC(true)
          .withSlot(NOT_HOLDING_ROBOT.slot);

  // The position it's trying to reach and stabilise at.
  @Logged(name = "Desired Position Inches", importance = Importance.INFO)
  private Distance m_desiredPosition = Inches.of(0.0);

  @Logged(name = "Neutral Mode", importance = Logged.Importance.DEBUG)
  private NeutralModeValue m_neutralMode =
      NeutralModeValue.Brake; // Coast: you let go, gravity lets it fall. Brake: locks it in place.

  private LinearFilter currentFilter = LinearFilter.singlePoleIIR(0.5, 0.02);

  // Climber Sim State:
  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_climberUnweightedSim =
      new ElevatorSim(
          CLIMBER.gearbox,
          CLIMBER.gearRatio,
          CLIMBER.NOT_HOLDING_ROBOT.kUnweightedCarriageMass.in(Kilograms),
          CLIMBER.kClimberDrumDiameter.div(2).in(Meters),
          CLIMBER.lowerLimit.in(Meters),
          CLIMBER.upperLimit.in(Meters),
          true,
          0);

  private final ElevatorSim m_climberWeightedSim =
      new ElevatorSim(
          CLIMBER.gearbox,
          CLIMBER.gearRatio,
          CLIMBER.HOLDING_ROBOT.kWeightedCarriageMass.in(Kilograms),
          CLIMBER.kClimberDrumDiameter.div(2).in(Meters),
          CLIMBER.lowerLimit.in(Meters),
          CLIMBER.upperLimit.in(Meters),
          true,
          0);

  private final TalonFXSimState m_motorSimState;

  /** Creates a new Climber. */
  public Climber() {
    // configuring the motor for the elevator:
    TalonFXConfiguration config = new TalonFXConfiguration();
    // Climbing No Robot PID Gains
    config.Slot0.kP = CLIMBER.NOT_HOLDING_ROBOT.kP;
    // config.Slot0.kD = CLIMBER.NOT_HOLDING_ROBOT.kD;
    // config.Slot0.kV = CLIMBER.NOT_HOLDING_ROBOT.kV;
    // config.Slot0.kA = CLIMBER.NOT_HOLDING_ROBOT.kA;
    config.Slot0.kG = CLIMBER.NOT_HOLDING_ROBOT.kG;
    config.Slot0.kS = CLIMBER.NOT_HOLDING_ROBOT.kS;
    config.Slot0.GravityType = CLIMBER.K_GRAVITY_TYPE_VALUE;

    // Climbing Robot PID Gains
    config.Slot1.kP = CLIMBER.HOLDING_ROBOT.kP;
    // config.Slot1.kD = CLIMBER.HOLDING_ROBOT.kD;
    // config.Slot1.kV = CLIMBER.HOLDING_ROBOT.kV;
    // config.Slot1.kA = CLIMBER.HOLDING_ROBOT.kA;
    config.Slot1.kS = CLIMBER.HOLDING_ROBOT.kS;
    config.Slot1.kG = CLIMBER.HOLDING_ROBOT.kG;
    config.Slot1.GravityType = CLIMBER.K_GRAVITY_TYPE_VALUE;

    config.Feedback.SensorToMechanismRatio =
        CLIMBER.gearRatio; // configNoRobotures climber to gear ratio. (check if absolute cancoder)
    config.MotionMagic.MotionMagicCruiseVelocity =
        CLIMBER.NOT_HOLDING_ROBOT.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = CLIMBER.NOT_HOLDING_ROBOT.motionMagicAcceleration;
    // config.MotionMagic.MotionMagicJerk = CLIMBER.NOT_HOLDING_ROBOT.motionMagicJerk; // TODO:
    // Implement Jerk when needed.
    config.CurrentLimits.StatorCurrentLimit =
        CLIMBER.kStatorCurrentLimit.in(
            Amps); // Prevents Climber from moving too jerkily and also breakage. TODO: Adjust this
    // value.
    config.CurrentLimits.StatorCurrentLimitEnable = true; // Enables previous function.
    // Sets limits on motor output. Seperate from current limits.
    config.MotorOutput.PeakReverseDutyCycle = CLIMBER.peakReverseOutput;
    config.MotorOutput.PeakForwardDutyCycle = CLIMBER.peakForwardOutput;
    config.MotorOutput.NeutralMode = m_neutralMode; // Puts the motor in Neutral mode.

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        convertDistancetoRotations(CLIMBER.upperLimit);
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        convertDistancetoRotations(CLIMBER.lowerLimit);

    // This is the function that applies all these configNoRoboturations to the motor.
    CtreUtils.configureTalonFx(m_climberMotor, config);

    m_motorSimState = m_climberMotor.getSimState();

    m_climberMotor.setPosition(Rotations.of(0));
  }

  @Logged(name = "Height Inches", importance = Logged.Importance.INFO)
  public double getHeightInches() {
    return getHeight().in(Inches);
  }

  @Logged(name = "Height Meters", importance = Logged.Importance.DEBUG)
  public Distance getHeight() {
    return CLIMBER.drumRotationsToDistance.times(
        m_climberMotor.getPosition().clone().refresh().getValue().magnitude());
  }

  // For Robot2d Simulation
  @NotLogged
  public LinearVelocity getVelocity() {
    return InchesPerSecond.of(
        m_climberMotor.getVelocity().clone().refresh().getValue().in(RotationsPerSecond)
            * CLIMBER.drumRotationsToDistance.in(Meters));
  }

  public double convertDistancetoRotations(Distance distance) {
    return distance.in(Meters) / CLIMBER.drumRotationsToDistance.in(Meters);
  }

  public void setDesiredPositionAndMotionMagicConfigs(
      Distance desiredPosition,
      double MotionMagicVelocity,
      double MotionMagicAcceleration,
      double Jerk) {
    m_request.Velocity = MotionMagicVelocity;
    m_request.Acceleration = MotionMagicAcceleration;
    m_request.Jerk = Jerk;
    m_desiredPosition =
        Meters.of(
            MathUtil.clamp(
                desiredPosition.in(Meters),
                CLIMBER.lowerLimit.in(Meters),
                CLIMBER.upperLimit.in(Meters)));
    m_climberMotor.setControl(m_request.withPosition(convertDistancetoRotations(desiredPosition)));
  }

  public void setPIDSlot(int slot) {
    m_request.Slot = slot;
  }

  @Logged(name = "Is in Holding Robot Mode?", importance = Importance.DEBUG)
  public boolean isInHoldingRobotMode() {
    return m_request.Slot == 1;
  }

  public Current getStatorCurrent() {
    return m_climberMotor.getStatorCurrent().clone().refresh().getValue();
  }

  @Logged(name = "Average Current", importance = Importance.DEBUG)
  public double getAverageCurrent() {
    return currentFilter.calculate(getStatorCurrent().in(Amps));
  }

  @Logged(name = "Is Holding Robot", importance = Importance.INFO)
  public boolean isHoldingRobot() {
    return getAverageCurrent() > CLIMBER.kCurrentHoldingRobotThreshold.in(Amps) && m_climberMotor.getVelocity().clone().refresh().getValue().in(RotationsPerSecond) < 1.0;
  }

  public void holdClimber() {
    if (isHoldingRobot()) {
      setDesiredPositionAndMotionMagicConfigs(
          getHeight(),
          CLIMBER.HOLDING_ROBOT.motionMagicCruiseVelocity,
          CLIMBER.HOLDING_ROBOT.motionMagicAcceleration,
          CLIMBER.HOLDING_ROBOT.motionMagicJerk);
      setPIDSlot(HOLDING_ROBOT.slot);
    } else {
      setDesiredPositionAndMotionMagicConfigs(
          getHeight(),
          CLIMBER.NOT_HOLDING_ROBOT.motionMagicCruiseVelocity,
          CLIMBER.NOT_HOLDING_ROBOT.motionMagicAcceleration,
          CLIMBER.NOT_HOLDING_ROBOT.motionMagicJerk);
      setPIDSlot(HOLDING_ROBOT.slot);
    }
  }

  private void updateSim(ElevatorSim sim) {
    sim.setInputVoltage(m_motorSimState.getMotorVoltage());

    sim.update(0.020);

    m_motorSimState.setRawRotorPosition(
        sim.getPositionMeters() * CLIMBER.gearRatio / CLIMBER.drumRotationsToDistance.in(Meters));
    m_motorSimState.setRotorVelocity(
        sim.getVelocityMetersPerSecond()
            * CLIMBER.gearRatio
            / CLIMBER.drumRotationsToDistance.in(Meters));
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    if (isHoldingRobot()) {
      updateSim(m_climberWeightedSim);
    } else {
      updateSim(m_climberUnweightedSim);
    }
  }
}
