package frc.team4201.lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;

public class FieldRectangle2d extends Rectangle2d {

  public FieldRectangle2d(Pose2d center, double xWidth, double yWidth) {
    super(center, xWidth, yWidth);
  }

  public FieldRectangle2d(Pose2d center, Distance xWidth, Distance yWidth) {
    super(center, xWidth, yWidth);
  }

  public FieldRectangle2d(Translation2d cornerA, Translation2d cornerB) {
    super(cornerA, cornerB);
  }

  public Translation2d[] getCorners() {
    ArrayList<Translation2d> corners = new ArrayList<>();
    corners.add(
        new Translation2d(
            getCenter().getMeasureX().plus(getMeasureXWidth().div(2.0)),
            getCenter().getMeasureY().plus(getMeasureYWidth().div(2.0))));
    corners.add(
        new Translation2d(
            getCenter().getMeasureX().plus(getMeasureXWidth().div(2.0)),
            getCenter().getMeasureY().minus(getMeasureYWidth().div(2.0))));
    corners.add(
        new Translation2d(
            getCenter().getMeasureX().minus(getMeasureXWidth().div(2.0)),
            getCenter().getMeasureY().minus(getMeasureYWidth().div(2.0))));
    corners.add(
        new Translation2d(
            getCenter().getMeasureX().minus(getMeasureXWidth().div(2.0)),
            getCenter().getMeasureY().plus(getMeasureYWidth().div(2.0))));
    return corners.toArray(new Translation2d[0]);
  }
}
