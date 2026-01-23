// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SHOOTERMOTORS.ManualRPS;
import frc.robot.Constants.SHOOTERMOTORS.Shot;
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
        distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 0.48));
    }
  private final ShooterRollers m_shooterRollers;

  private final ShooterHood m_shooterHood;

  private final ManualRPS m_rps;
  private final Angle m_angle;

  public Shoot(
      ShooterRollers shooterRollers, ShooterHood shooterHood, ManualRPS rps, Angle angle) {
    m_shooterRollers = shooterRollers;
    m_rps = rps;
    m_shooterHood = shooterHood;
    m_angle = angle;

    addRequirements(shooterRollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        final Distance distanceToHub = getDistanceToHub();
        final Shot shot = distanceToShotMap.get(distanceToHub);
        m_shooterRollers.setRPMOutputFOC(shot.shooterRPM);
        m_shooterHood.setPosition(shot.hoodPosition);
        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterRollers.setRPSOutputFOC(ManualRPS.IDLE.getRPS());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
