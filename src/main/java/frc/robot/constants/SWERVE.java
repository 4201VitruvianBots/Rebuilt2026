package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
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

  public static final PIDConstants kTranslationPID = new PIDConstants(10, 0, 0);
  public static final PIDConstants kRotationPID = new PIDConstants(7, 0, 0);

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

  public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
  public static final double kMaxRotationRadiansPerSecond =
      Math.PI * 0.3; // Temporary to reduce speed (original value 2.0).
}
