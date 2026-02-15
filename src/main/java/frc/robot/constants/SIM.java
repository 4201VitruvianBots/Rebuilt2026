package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.derive;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.DistanceUnit;

public class SIM {
  // For some reason, the line width value needs to be 2 times larger (~12 times larger in Glass)
  // in order to actually match up with the visual thickness of lines in the Mechanism2d
  public static final DistanceUnit LineWidthInches =
      derive(Inches).splitInto(2).named("LineWidthInches").symbol("lw in").make();

  public static final Translation3d intakeOrigin = new Translation3d(0.06, 0, 0.13);
  public static final Translation3d hopperOrigin = new Translation3d(0.0, 0.0, 0.0);

  public static int MAX_FUEL = 48;
}
