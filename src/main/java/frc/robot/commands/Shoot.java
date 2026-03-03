package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FIELD;
import frc.robot.constants.FLYWHEEL.Shot;
import frc.robot.constants.SWERVE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

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
                  Degrees.of(
                      Interpolator.forDouble()
                          .interpolate(
                              startValue.hoodAngle.in(Degrees), endValue.hoodAngle.in(Degrees), t)),
                  Interpolator.forDouble()
                      .interpolate(startValue.timeOfFlight, endValue.timeOfFlight, t)));

  static {
    // TODO: Make at least 20 values for this. Yes. 20. Ideally 30
    distanceToShotMap.put(
        Meters.of(1.391736631), new Shot(RPM.of(1470), Degrees.of(9.0), 1.286)); //
    distanceToShotMap.put(Meters.of(1.45650895207174), new Shot(RPM.of(1470), Degrees.of(10.0), 1.286)); // Tuned
    distanceToShotMap.put(Meters.of(1.9791021529687), new Shot(RPM.of(1530), Degrees.of(11.0), 1.286)); // Tuned
    distanceToShotMap.put(Meters.of(2.0749597158415), new Shot(RPM.of(1600), Degrees.of(13.3), 1.286)); // Tuned
    distanceToShotMap.put(Meters.of(2.5916617555783), new Shot(RPM.of(1600), Degrees.of(14.5), 1.286)); // Tuned
    distanceToShotMap.put(Meters.of(3.01901001108563), new Shot(RPM.of(1650), Degrees.of(14.7), 1.286));
    distanceToShotMap.put(Meters.of(3.31521515713456), new Shot(RPM.of(1670), Degrees.of(22), 1.286)); // Half Tuned
    distanceToShotMap.put(Meters.of(3.44235025242724), new Shot(RPM.of(1678), Degrees.of(21), 1.286));
    distanceToShotMap.put(Meters.of(3.55309150390832), new Shot(RPM.of(1710), Degrees.of(19.8), 1.286));
    distanceToShotMap.put(Meters.of(3.79704412351433), new Shot(RPM.of(1720), Degrees.of(20), 1.286));
    distanceToShotMap.put(Meters.of(4.6722084908365), new Shot(RPM.of(2000), Degrees.of(21), 1.286));
    distanceToShotMap.put(Meters.of(5.44820580711993), new Shot(RPM.of(2100), Degrees.of(24.5), 1.286));
    distanceToShotMap.put(Meters.of(2.13), new Shot(RPM.of(1500), Degrees.of(5), 1.286));
    distanceToShotMap.put(Meters.of(2.13), new Shot(RPM.of(1500), Degrees.of(5), 1.286));
    distanceToShotMap.put(Meters.of(2.13), new Shot(RPM.of(1500), Degrees.of(5), 1.286));
    distanceToShotMap.put(Meters.of(2.13), new Shot(RPM.of(1500), Degrees.of(5), 1.286));
    distanceToShotMap.put(Meters.of(2.13), new Shot(RPM.of(1500), Degrees.of(5), 1.286));
    distanceToShotMap.put(Meters.of(2.13), new Shot(RPM.of(1500), Degrees.of(5), 1.286));
    distanceToShotMap.put(Meters.of(2.13), new Shot(RPM.of(1500), Degrees.of(5), 1.286));
    distanceToShotMap.put(Meters.of(2.13), new Shot(RPM.of(1500), Degrees.of(5), 1.286));
    distanceToShotMap.put(Meters.of(2.13), new Shot(RPM.of(1500), Degrees.of(5), 1.286));
  }

  private final Vision m_vision;
  private final Flywheel m_flywheel;
  private final CommandSwerveDrivetrain m_swerveDrivetrain;
  private DoubleSupplier m_throttleInput = () -> 0.0;
  private DoubleSupplier m_strafeInput = () -> 0.0;
  private static double phaseDelay = 0.0;
  private Rotation2d lastDriveAngle;
  private double lastHoodAngle;

  private Rotation2d launcherRotation = new Rotation2d(Degrees.of(0.0));
  private Transform2d robotToLauncher =
      new Transform2d(Meters.of(0.0), Meters.of(0.0), launcherRotation);

  private final Hood m_shooterHood;

  Translation2d m_goal = new Translation2d();

  /** Shoot on the move command */
  public Shoot(
      Flywheel flywheel,
      Hood shooterHood,
      Vision vision,
      CommandSwerveDrivetrain swerveDrive,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput) {
    m_flywheel = flywheel;
    m_swerveDrivetrain = swerveDrive;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_shooterHood = shooterHood;
    m_vision = vision;

    addRequirements(flywheel, shooterHood, swerveDrive);
    SmartDashboard.putData(this);
  }

  /** Standard shooting without shoot on the move capabilities. Used only in auto. */
  public Shoot(
      Flywheel flywheel,
      Hood shooterHood,
      Uptake uptake,
      Vision vision,
      CommandSwerveDrivetrain swerveDrive) {
    m_flywheel = flywheel;
    m_shooterHood = shooterHood;
    m_vision = vision;
    m_swerveDrivetrain = swerveDrive;

    addRequirements(flywheel, shooterHood, uptake);
    SmartDashboard.putData(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_goal = FIELD.HUB.GOAL.getTargetPosition().toTranslation2d();
    if (RobotBase.isReal()) phaseDelay = 0.03;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = m_swerveDrivetrain.getState().Pose;
    ChassisSpeeds robotRelativeVelocity = m_swerveDrivetrain.getState().Speeds;
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from launcher to m_goal
    Pose2d launcherPosition = estimatedPose.transformBy(robotToLauncher);
    double launcherToTargetDistance = m_goal.getDistance(launcherPosition.getTranslation());
    Shot shot = distanceToShotMap.get(Meters.of(launcherToTargetDistance));

    // Calculate field relative launcher velocity
    // This isn't actually the launcherVelocity given it won't account for angular velocity of robot
    double launcherVelocityX = robotRelativeVelocity.vxMetersPerSecond;
    double launcherVelocityY = robotRelativeVelocity.vyMetersPerSecond;

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight = shot.timeOfFlight;
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;
    shot = distanceToShotMap.get(Meters.of(lookaheadLauncherToTargetDistance));

    for (int i = 0; i < 20; i++) {
      timeOfFlight = shot.timeOfFlight;
      double offsetX = launcherVelocityX * timeOfFlight;
      double offsetY = launcherVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = m_goal.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    Rotation2d driveAngle = m_goal.minus(lookaheadPose.getTranslation()).getAngle();

    double hoodAngle = shot.hoodAngle.in(Radians);

    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    lastHoodAngle = hoodAngle;
    // all of the logic for angle is above this Comment
    m_flywheel.setRPMOutputFOC(shot.shooterRPM);
    m_shooterHood.setAngle(Radians.of(hoodAngle));
    
    if (Controls.isRedAlliance()){
      m_swerveDrivetrain.setChassisSpeedsWithHeading(
        SWERVE.kMaxSpeed.times(m_throttleInput.getAsDouble()),
        SWERVE.kMaxSpeed.times(m_strafeInput.getAsDouble()),
        driveAngle.rotateBy(Rotation2d.k180deg));
    } else {
      m_swerveDrivetrain.setChassisSpeedsWithHeading(
        SWERVE.kMaxSpeed.times(m_throttleInput.getAsDouble()),
        SWERVE.kMaxSpeed.times(m_strafeInput.getAsDouble()),
        driveAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.setTorqueCurrentOutputFOC(Volts.of(0.0));
    m_flywheel.setRPMOutputFOC(RPM.of(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
