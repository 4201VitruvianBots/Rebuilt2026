package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FLYWHEEL.Shot;
import frc.robot.Constants.SWERVE;
import frc.robot.constants.FIELD;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

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
    distanceToShotMap.put(Meters.of(3.048), new Shot(RPM.of(2200), Degrees.of(73), 1.16));
    distanceToShotMap.put(Meters.of(6.00), new Shot(RPM.of(2900), Degrees.of(70), 1.2));
  }

  private final Flywheel m_flywheel;
  private final Vision m_vision;
  private final CommandSwerveDrivetrain m_swerveDrivetrain;
  private final DoubleSupplier m_throttleInput;
  private final DoubleSupplier m_strafeInput;
  private static double phaseDelay; 

  private Double kTeleP_Theta = 3.0;
  private Double kTeleD_Theta = 0.0;
  public static final double kTeleI_Theta = 0.0;

  private PIDController m_PidController =
      new PIDController(kTeleP_Theta, kTeleI_Theta, kTeleD_Theta);

  private final Hood m_shooterHood;

  Translation2d m_goal = new Translation2d();

  public ShootOnTheMove(
      Flywheel flywheel,
      Hood shooterHood,
      Vision vision,
      CommandSwerveDrivetrain swerveDrive,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput) {
    m_flywheel = flywheel;
    m_vision = vision;
    m_swerveDrivetrain = swerveDrive;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_shooterHood = shooterHood;

    addRequirements(flywheel, shooterHood, swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_goal = FIELD.HUB.GOAL.getTargetPosition().toTranslation2d();
    m_PidController.enableContinuousInput(-Math.PI, Math.PI);
    m_PidController.reset();
    phaseDelay = 0.03; // Magic number for now, figure out how to calculate this later
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shot shot = distanceToShotMap.get(m_vision.getDistanceToHub()); // Create a shot object based on hub distance.
    var chassisSpeeds = m_swerveDrivetrain.getState().Speeds; // Get robot relative chassis speeds
    Pose2d estimatedPose = m_swerveDrivetrain.getState().Pose.exp( 
            new Twist2d(
                chassisSpeeds.vxMetersPerSecond * phaseDelay,
                chassisSpeeds.vyMetersPerSecond * phaseDelay,
                chassisSpeeds.omegaRadiansPerSecond * phaseDelay)); // Account for where the pose should be 
 
    var fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, estimatedPose.getRotation());
    double VelocityY = fieldRelativeSpeeds.vyMetersPerSecond;
    double VelocityX = fieldRelativeSpeeds.vxMetersPerSecond;

    double timeOfFlight = shot.timeOfFlight;

    Translation2d movingGoalLocation = m_goal.minus(new Translation2d(VelocityX, VelocityY).times(timeOfFlight));

    System.out.println(movingGoalLocation.getMeasureX().in(Meters));
    System.out.println(movingGoalLocation.getMeasureY().in(Meters));

    Translation2d toMovingGoal = movingGoalLocation.minus(estimatedPose.getTranslation());

    double newDist = toMovingGoal.getDistance(new Translation2d());

    var targetDelta = toMovingGoal.getAngle();
    shot = distanceToShotMap.get(Meters.of(newDist));
    timeOfFlight = shot.timeOfFlight;

    // all of the logic for angle is above this Comment

    var turnRate =
        m_PidController.calculate(
            estimatedPose.getRotation().getRadians(),
            targetDelta.getRadians());

    m_flywheel.setRPMOutputFOC(shot.shooterRPM);
    m_shooterHood.setAngle(shot.hoodAngle);

    m_swerveDrivetrain.setChassisSpeedControl(
        new ChassisSpeeds(
            m_throttleInput.getAsDouble() * SWERVE.kMaxSpeedShootingMetersPerSecond,
            m_strafeInput.getAsDouble() * SWERVE.kMaxSpeedShootingMetersPerSecond,
            turnRate));
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
