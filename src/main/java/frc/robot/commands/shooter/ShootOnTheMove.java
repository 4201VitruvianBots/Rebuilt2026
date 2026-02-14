package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.FLYWHEEL.Shot;
import frc.robot.Constants.SWERVE;
import frc.robot.constants.FIELD;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class ShootOnTheMove extends Command {
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
    // TODO: Make at least 20 values for this. Yes. 20.
    distanceToShotMap.put(
        Meters.of(1.8086638318064376),
        new Shot(RPM.of(2175), Degrees.of(75), 0.96399)); // Hood position is a placeholder
    distanceToShotMap.put(Meters.of(3.048), new Shot(RPM.of(2200), Degrees.of(73), 1.286));
    distanceToShotMap.put(Meters.of(6.00), new Shot(RPM.of(2900), Degrees.of(70), 1.4));
  }

  private final Flywheel m_flywheel;
  private final CommandSwerveDrivetrain m_swerveDrivetrain;
  private final DoubleSupplier m_throttleInput;
  private final DoubleSupplier m_strafeInput;
  private static double phaseDelay = 0.0;
  private Rotation2d lastDriveAngle;
  private double lastHoodAngle;

  private final LinearFilter driveAngleFilter =
    LinearFilter.movingAverage((int) (0.1 / 0.02));

  private Rotation2d launcherRotation = new Rotation2d(Degrees.of(0.0));
  private Transform2d robotToLauncher = new Transform2d(Meters.of(0.0), Meters.of(0.0), launcherRotation);

  private Double kTeleP_Theta = 7.0;
  private Double kTeleD_Theta = 0.0;
  public static final double kTeleI_Theta = 0.0;

  private ProfiledPIDController m_PidController; 

  private final Hood m_shooterHood;

  Translation2d m_goal = new Translation2d();

  public ShootOnTheMove(
      Flywheel flywheel,
      Hood shooterHood,
      CommandSwerveDrivetrain swerveDrive,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput) {
    m_flywheel = flywheel;
    m_swerveDrivetrain = swerveDrive;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_shooterHood = shooterHood;

    addRequirements(flywheel, shooterHood, swerveDrive);
    SmartDashboard.putData(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_goal = FIELD.HUB.GOAL.getTargetPosition().toTranslation2d();
    m_PidController = new ProfiledPIDController(kTeleP_Theta, kTeleI_Theta, kTeleD_Theta, new Constraints(SWERVE.kMaxSpeedMetersPerSecond, SWERVE.kMaxAccelerationShootingMetersPerSecond));
    m_PidController.enableContinuousInput(-Math.PI, Math.PI);
    m_PidController.setTolerance(1.0);
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
    Rotation2d driveAngle =
        m_goal.minus(lookaheadPose.getTranslation()).getAngle();
    double hoodAngle = shot.hoodAngle.in(Radians);

    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    lastHoodAngle = hoodAngle;
    double driveVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / 0.02);
          
    // all of the logic for angle is above this Comment

    var turnRate =
        m_PidController.calculate(
            estimatedPose.getRotation().getRadians(), new State(
                      driveAngle.getRadians(),
                      driveVelocity));

    m_flywheel.setRPMOutputFOC(shot.shooterRPM);
    m_shooterHood.setAngle(Radians.of(hoodAngle));

    double omegaOutput = m_PidController.getSetpoint().velocity + turnRate;
    m_swerveDrivetrain.setChassisSpeedControl(
        new ChassisSpeeds(
            m_throttleInput.getAsDouble() * SWERVE.kMaxSpeedShootingMetersPerSecond,
            m_strafeInput.getAsDouble() * SWERVE.kMaxSpeedShootingMetersPerSecond,
            omegaOutput));
    lastDriveAngle = driveAngle;
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
