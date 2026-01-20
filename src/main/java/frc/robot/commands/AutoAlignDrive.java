// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.constants.FIELD;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import java.util.function.DoubleSupplier;

public class AutoAlignDrive extends Command {
  private final CommandSwerveDrivetrain m_swerveDrivetrain;
  Translation2d m_goal = new Translation2d();

  public static final double kTeleP_Theta = 10.0;
  public static final double kTeleI_Theta = 0.0;
  public static final double kTeleD_Theta = 0.0;

  private final PIDController m_PidController =
      new PIDController(kTeleP_Theta, kTeleI_Theta, kTeleD_Theta);

  private final DoubleSupplier m_throttleInput;
  private final DoubleSupplier m_turnInput;

  /** Creates a new AutoAlign. */
  public AutoAlignDrive(
      CommandSwerveDrivetrain commandSwerveDrivetrain,
      DoubleSupplier throttleInput,
      DoubleSupplier turnInput) {
    m_swerveDrivetrain = commandSwerveDrivetrain;
    m_throttleInput = throttleInput;
    m_turnInput = turnInput;
    m_PidController.setTolerance(Units.degreesToRadians(2));
    m_PidController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PidController.reset();
    if (Controls.isBlueAlliance()) {
      m_goal = FIELD.blueAutoHub;
    } else {
      m_goal = FIELD.redAutoHub;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var setPoint = m_goal.minus(m_swerveDrivetrain.getState().Pose.getTranslation());
    var turnRate =
        m_PidController.calculate(
            m_swerveDrivetrain.getState().Pose.getRotation().getRadians(),
            setPoint.getAngle().getRadians());
    m_swerveDrivetrain.setChassisSpeedControl(
        new ChassisSpeeds(
            m_throttleInput.getAsDouble() * SWERVE.kMaxSpeedMetersPerSecond,
            m_turnInput.getAsDouble() * SWERVE.kMaxSpeedMetersPerSecond,
            turnRate));
    System.out.println(turnRate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
