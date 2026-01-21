// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CAN;
import frc.robot.Constants.INTAKEMOTORS.PIVOT;
import frc.team4201.lib.utils.CtreUtils;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
  @Logged(name = "Intake Pivot Motor", importance = Importance.INFO)
  private final TalonFX m_motor = new TalonFX(CAN.kIntakePivotMotor);

  private final CANcoder m_canCoder = new CANcoder(CAN.kPivotEncoder);
  
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0.0).withEnableFOC(true); 

  private Angle m_desiredRotation = PIVOT.PIVOT_SETPOINT.STOWED.get();

    // Simulation Code
  private final SingleJointedArmSim m_Sim =
      new SingleJointedArmSim(
          PIVOT.gearbox,
          PIVOT.gearRatio,
          SingleJointedArmSim.estimateMOI(PIVOT.baseLength.in(Meters), PIVOT.mass.in(Kilograms)),
          PIVOT.baseLength.in(Meters),
          PIVOT.minAngle.in(Radians),
          PIVOT.maxAngle.in(Radians),
          false,
          PIVOT.startingAngle.in(Radians));

  public IntakePivot() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = PIVOT.kP;
    config.Slot0.kD = PIVOT.kD;
    config.Slot0.kA = PIVOT.kA;
    config.Slot0.kV = PIVOT.kV;
    config.Slot0.kS = PIVOT.kS;
    config.Slot0.GravityType = PIVOT.K_GRAVITY_TYPE_VALUE;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = PIVOT.gearRatio;
    config.MotorOutput.PeakForwardDutyCycle = PIVOT.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = PIVOT.peakReverseOutput;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.MotionMagic.MotionMagicCruiseVelocity = PIVOT.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = PIVOT.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = PIVOT.motionMagicJerk;

    CtreUtils.configureTalonFx(m_motor, config);
  }

    public void setPosition(Angle rotations) {
      m_desiredRotation = Degrees.of(MathUtil.clamp(rotations.in(Degrees), PIVOT.minAngle.in(Degrees), PIVOT.maxAngle.in(Degrees)));
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
