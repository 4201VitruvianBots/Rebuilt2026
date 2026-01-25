// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SHOOTERMOTORS.ManualRPM;
import frc.robot.Constants.SHOOTERMOTORS.Shot;
import frc.robot.constants.FIELD;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterRollers;

public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
      private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
   );

    static {
        distanceToShotMap.put(Inches.of(52.0), new Shot(2800, 0.19));
        distanceToShotMap.put(Inches.of(149.4), new Shot(2900, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new Shot(3000, 0.48));
    }
  private final ShooterRollers m_shooterRollers;

  // private final ShooterHood m_shooterHood;
  private final CommandSwerveDrivetrain m_swerveDrivetrain;

  Translation2d m_goal = new Translation2d();


  public Shoot(CommandSwerveDrivetrain swerve,
      ShooterRollers shooterRollers) {
    m_shooterRollers = shooterRollers;
    // m_shooterHood = shooterHood;
    m_swerveDrivetrain = swerve;

    addRequirements(shooterRollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Controls.isBlueAlliance()) {
      m_goal = FIELD.blueAutoHub;
    } else {
      m_goal = FIELD.redAutoHub;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Distance distanceToHub = Meters.of(m_swerveDrivetrain.getState().Pose.getTranslation().getDistance(m_goal));
    final Shot shot = distanceToShotMap.get(distanceToHub);
    m_shooterRollers.setRPMOutputFOC(shot.shooterRPM);
    // m_shooterHood.setPosition(shot.hoodPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterRollers.setVoltageOutputFOC(Volts.of(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
