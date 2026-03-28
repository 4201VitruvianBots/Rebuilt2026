package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import frc.team4201.lib.utils.ModuleMap.MODULE_POSITION;
import java.util.Map;

public class SWERVE {
  public enum MOTOR_TYPE {
    ALL,
    DRIVE,
    STEER
  }

  public static final Distance kWheelBase = Inches.of(17.75);
  public static final Distance kTrackWidth = Inches.of(24.5);
  public static final Distance kBumperHeight = Inches.of(4.5);
  public static final Distance kBumperWidth = Inches.of(4.0);

  public static final Distance kChassisWidth = Inches.of(30.5);
  public static final Distance kChassisLength = Inches.of(23.0);

  public static final Map<MODULE_POSITION, Translation2d> kModuleTranslations =
      Map.of(
          MODULE_POSITION.FRONT_LEFT,
          new Translation2d(kWheelBase.div(2).in(Meters), kTrackWidth.div(2).in(Meters)),
          MODULE_POSITION.FRONT_RIGHT,
          new Translation2d(kWheelBase.div(2).in(Meters), -kTrackWidth.div(2).in(Meters)),
          MODULE_POSITION.BACK_LEFT,
          new Translation2d(-kWheelBase.div(2).in(Meters), kTrackWidth.div(2).in(Meters)),
          MODULE_POSITION.BACK_RIGHT,
          new Translation2d(-kWheelBase.div(2).in(Meters), -kTrackWidth.div(2).in(Meters)));

  public static final LinearVelocity kMaxSpeed = FeetPerSecond.of(18);
  // Made up values
  public static final LinearVelocity kMaxSpeedBump = FeetPerSecond.of(5.5);
  public static final LinearAcceleration kMaxAccelerationBump = FeetPerSecondPerSecond.of(3.0);
  // In units, where one unit is the max speed of the robot because 100% output of the joystick will translate to max speed.
  public static final double kXAccelRateLimit = 10.0;
  public static final double kYAccelRateLimit = 10.0;
  public static final double kXDeccelRateLimit = -3.0;
  public static final double kYDeccelRateLimit = -3.0;

  public static final LinearVelocity kMaxSpeedShooting = FeetPerSecond.of(8.0);
  public static final LinearAcceleration kMaxAccelerationShooting = FeetPerSecondPerSecond.of(11.0);
  public static final AngularVelocity kMaxRotation =
      RotationsPerSecond.of(Math.PI * 0.3); // Temporary to reduce speed (original value 2.0).

  // Constants needed for auto align
  // TODO: Figure out where these numbers are coming from and if we need to change them.
  public class AUTO_ALIGN {
    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
    public static final Distance kPositionTolerance = Inches.of(0.4);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(0.25);
    public static final Time kEndTriggerDebounce = Seconds.of(0.04);
    public static final Time kAlignmentAdjustmentTimeout = Seconds.of(0.075);
    public static final PIDConstants kAutoAlignTranslationPID = new PIDConstants(17.0, 0.0, 0.0);
    public static final PIDConstants kAutoAlignRotationPID = new PIDConstants(9.0, 0.0, 0.0);

    public static final PPHolonomicDriveController kDriveController =
        new PPHolonomicDriveController(kAutoAlignTranslationPID, kAutoAlignRotationPID);

    public static final PathConstraints kAutoAlignPathConstraints =
        new PathConstraints(1.75, 1.25, 1.0 / 2 * Math.PI, 1 * Math.PI);
  }
}
