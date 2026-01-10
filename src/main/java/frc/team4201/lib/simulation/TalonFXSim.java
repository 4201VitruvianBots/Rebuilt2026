package frc.team4201.lib.simulation;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class TalonFXSim {

  enum MODEL_TYPE {
    ARM,
    ELEVATOR,
    SIMPLE_MOTOR
  }

  private final TalonFX m_motor;
  private final TalonFXSimState m_simState;
  private final LinearSystemSim<?, ?, ?> m_model;
  private final MODEL_TYPE m_modelType;
  private Per<AngleUnit, DistanceUnit> m_conversionFactor;
  private Dimensionless m_conversionFactor2;

  public TalonFXSim(TalonFX motor, LinearSystemSim<?, ?, ?> model) {
    this(motor, model, Rotations.of(1).div(Inches.of(1)));
  }

  /**
   * TODO: Hide functions based on implementation/simplify code implementation to account for
   * Dimensionless
   */
  public TalonFXSim(
      TalonFX motor,
      LinearSystemSim<?, ?, ?> model,
      Per<AngleUnit, DistanceUnit> conversionFactor) {
    m_motor = motor;
    m_simState = motor.getSimState();
    m_model = model;
    m_modelType = MODEL_TYPE.ELEVATOR;
    m_conversionFactor = conversionFactor;
  }

  public TalonFXSim(TalonFX motor, LinearSystemSim<?, ?, ?> model, Dimensionless conversionFactor) {
    m_motor = motor;
    m_simState = motor.getSimState();
    m_model = model;
    m_modelType = MODEL_TYPE.SIMPLE_MOTOR;
    m_conversionFactor2 = conversionFactor;
  }

  public TalonFXSim withConversionFactor(Per<AngleUnit, DistanceUnit> conversionFactor) {
    m_conversionFactor = conversionFactor;
    return this;
  }

  public TalonFXSim withConversionFactor(Dimensionless conversionFactor) {
    m_conversionFactor2 = conversionFactor;
    return this;
  }

  private void setInputVoltage(double volts) {
    m_model.setInput(new double[] {volts});
  }

  public void update() {
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    setInputVoltage(m_simState.getMotorVoltage());

    m_model.update(0.02);

    switch (m_modelType) {
      case ARM -> {
        // Radians
        m_simState.setRawRotorPosition(Radians.of(m_model.getOutput(0)).times(m_conversionFactor2));
        m_simState.setRotorVelocity(
            RadiansPerSecond.of(m_model.getOutput(1)).times(m_conversionFactor2));
      }
      case ELEVATOR -> {
        // Meters
        var position = Meters.of(m_model.getOutput(0));
        var velocity = MetersPerSecond.of(m_model.getOutput(1));
        //                m_simState.setRawRotorPosition(position.times(m_conversionFactor));
        //                m_simState.setRotorVelocity(velocity.times(m_conversionFactor));
      }
      case SIMPLE_MOTOR -> {
        // Radians
        m_simState.setRawRotorPosition(Radians.of(m_model.getOutput(0)).times(m_conversionFactor2));
        m_simState.setRotorVelocity(
            RadiansPerSecond.of(m_model.getOutput(1)).times(m_conversionFactor2));
        // Matrix<N1, N1> accel =
        // m_model.m_plant.getA().times(this.m_x).plus(this.m_plant.getB().times(this.m_u))
        // m_simState.setRotorAcceleration();
      }
    }
  }
}
