package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FIELD;
import frc.robot.constants.FLYWHEEL.Shot;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Vision;

public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap =
      new InterpolatingTreeMap<>(
          (startValue, endValue, q) ->
              InverseInterpolator.forDouble()
                  .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
          (startValue, endValue, t) ->
              new Shot(
                  RPM.of(
                      Interpolator.forDouble()
                          .interpolate(
                              startValue.shooterRPM.in(RPM), endValue.shooterRPM.in(RPM), t)),
                  Rotations.of(
                      Interpolator.forDouble()
                          .interpolate(
                              startValue.hoodAngle.in(Rotations),
                              endValue.hoodAngle.in(Rotations),
                              t))));

  static {
    distanceToShotMap.put(
        Meters.of(1.8086638318064376),
        new Shot(RPM.of(2175), Rotations.of(0.40))); // Hood position is a placeholder
    distanceToShotMap.put(Meters.of(3.42), new Shot(RPM.of(2200), Rotations.of(0.19)));
    distanceToShotMap.put(Meters.of(6.00), new Shot(RPM.of(2300), Rotations.of(0.15)));
  }

  private final Flywheel m_flywheel;
  private final Vision m_vision;

  private final Hood m_shooterHood;

  Translation2d m_goal = new Translation2d();

  public Shoot(Flywheel flywheel, Vision vision, Hood shooterHood) {
    m_flywheel = flywheel;
    m_vision = vision;
    m_shooterHood = shooterHood;

    addRequirements(flywheel, shooterHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_goal = FIELD.HUB.GOAL.getTargetPosition().toTranslation2d();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shot shot = distanceToShotMap.get(m_vision.getDistanceToHub());
    m_flywheel.setRPMOutputFOC(shot.shooterRPM);
    // m_shooterHood.setPosition(shot.hoodPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.setTorqueCurrentOutputFOC(Volts.of(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
