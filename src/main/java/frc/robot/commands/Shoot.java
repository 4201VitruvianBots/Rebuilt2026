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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FIELD;
import frc.robot.constants.FLYWHEEL;
import frc.robot.constants.FLYWHEEL.Shot;
import frc.robot.constants.SWERVE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
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
    // Everything has been offset by plus 5.5 degrees.
    distanceToShotMap.put(Meters.of(1.10695), new Shot(RPM.of(1260 + 30), Degrees.of(0), 0.9)); //
    distanceToShotMap.put(
        Meters.of(2.0749597158415), new Shot(RPM.of(1540 + 30), Degrees.of(0), 1.1)); // Tuned
    distanceToShotMap.put(
        Meters.of(2.31209), new Shot(RPM.of(1553 + 30), Degrees.of(0.4), 1.1)); // Tuned
    distanceToShotMap.put(
        Meters.of(2.5916617555783), new Shot(RPM.of(1570 + 30), Degrees.of(2.2), 1.1)); // Tuned
    distanceToShotMap.put(
        Meters.of(3.152353828396097), new Shot(RPM.of(1648 + 30), Degrees.of(3.5), 1.1)); // Tuned
    distanceToShotMap.put(
        Meters.of(3.97453), new Shot(RPM.of(1764.3 + 30), Degrees.of(6.234), 1.1)); // Tuned
    distanceToShotMap.put(Meters.of(4.2697), new Shot(RPM.of(1764.3 + 30), Degrees.of(8), 1.16));
    distanceToShotMap.put(
        Meters.of(5.44820580711993), new Shot(RPM.of(1945 + 30), Degrees.of(14), 1.16));
  }

  private final Vision m_vision;
  private final Flywheel m_flywheel;
  private final CommandSwerveDrivetrain m_swerveDrivetrain;
  private CommandXboxController m_driverController;
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

  public static Shot getShotForDistance(Distance distance) {
    return distanceToShotMap.get(distance);
  }

  /** Shoot on the move command with rumble */
  public Shoot(
      Flywheel flywheel,
      Hood shooterHood,
      Vision vision,
      CommandXboxController driverController,
      CommandSwerveDrivetrain swerveDrive,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput) {
    m_flywheel = flywheel;
    m_driverController = driverController;
    m_swerveDrivetrain = swerveDrive;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_shooterHood = shooterHood;
    m_vision = vision;

    addRequirements(flywheel, shooterHood);
    SmartDashboard.putData(this);
  }

  /** Standard shooting without shoot on the move capabilities. Used only in auto. */
  public Shoot(
      Flywheel flywheel, Hood shooterHood, Vision vision, CommandSwerveDrivetrain swerveDrive) {
    m_flywheel = flywheel;
    m_shooterHood = shooterHood;
    m_vision = vision;
    m_swerveDrivetrain = swerveDrive;

    addRequirements(flywheel, shooterHood);
    SmartDashboard.putData(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_goal = FIELD.TARGET.CURRENT_TARGET.getTargetPosition().toTranslation2d();

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
    double effectiveTimeOfFlight = timeOfFlight;
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      shot = distanceToShotMap.get(Meters.of(lookaheadLauncherToTargetDistance));
      timeOfFlight = shot.timeOfFlight;
      effectiveTimeOfFlight =
          (1 - Math.pow(Math.E, -(FLYWHEEL.kFuelDragCoefficient * timeOfFlight)))
              / FLYWHEEL
                  .kFuelDragCoefficient; // Accounting for linear drag. This is the equation for
      // continuous decay.
      // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
      double offsetX = launcherVelocityX * effectiveTimeOfFlight;
      double offsetY = launcherVelocityY * effectiveTimeOfFlight;
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
    m_flywheel.setRPMOutput(shot.shooterRPM);
    m_shooterHood.setAngle(Radians.of(hoodAngle));
    if (m_flywheel.isAtRPMsetpoint()) {
      if (m_driverController != null)
        m_driverController.setRumble(
            RumbleType.kBothRumble, FLYWHEEL.kRumbleStrength); // Null in auto
    } else {
      if (m_driverController != null)
        m_driverController.setRumble(RumbleType.kBothRumble, 0); // Null in auto
    }

    Rotation2d moduleAngle = new Rotation2d();
    moduleAngle = m_swerveDrivetrain.getState().clone().ModuleStates[0].angle;
    double moduleAngleDelta = 54 - Math.abs(moduleAngle.getDegrees()); // Since our drivetrain is square, swerve drive brake is 53.75 degrees instead of 45
    int moduleAngleDeltaInt = (int) Math.round(moduleAngleDelta);
    boolean isBraking = moduleAngleDeltaInt == 0;

    if (!isBraking){
      m_swerveDrivetrain.setChassisSpeedsWithHeading(
        SWERVE.kMaxSpeed.times(m_throttleInput.getAsDouble()),
        SWERVE.kMaxSpeed.times(m_strafeInput.getAsDouble()),
        Controls.isRedAlliance() ? driveAngle.rotateBy(Rotation2d.k180deg) : driveAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_driverController != null)
      m_driverController.setRumble(RumbleType.kBothRumble, 0); // Null in auto
    m_flywheel.setVoltageOutput(Volts.of(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
