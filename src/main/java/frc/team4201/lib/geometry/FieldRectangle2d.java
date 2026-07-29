package frc.team4201.lib.geometry;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rectangle2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Distance;
import java.util.ArrayList;

public class FieldRectangle2d {
  Rectangle2d rectangle2d;

  public FieldRectangle2d(Pose2d center, double xWidth, double yWidth) {
    rectangle2d = new Rectangle2d(center, xWidth, yWidth);
  }

  public FieldRectangle2d(Pose2d center, Distance xWidth, Distance yWidth) {
    rectangle2d = new Rectangle2d(center, xWidth, yWidth);
  }

  public FieldRectangle2d(Translation2d cornerA, Translation2d cornerB) {
    rectangle2d = new Rectangle2d(cornerA, cornerB);
  }

  public Rectangle2d get() {
    return rectangle2d;
  }

  public Translation2d[] getCorners() {
    ArrayList<Translation2d> corners = new ArrayList<>();
    corners.add(
        new Translation2d(
            rectangle2d.getCenter().getMeasureX().plus(rectangle2d.getMeasureXWidth().div(2.0)),
            rectangle2d.getCenter().getMeasureY().plus(rectangle2d.getMeasureYWidth().div(2.0))));
    corners.add(
        new Translation2d(
            rectangle2d.getCenter().getMeasureX().plus(rectangle2d.getMeasureXWidth().div(2.0)),
            rectangle2d.getCenter().getMeasureY().minus(rectangle2d.getMeasureYWidth().div(2.0))));
    corners.add(
        new Translation2d(
            rectangle2d.getCenter().getMeasureX().minus(rectangle2d.getMeasureXWidth().div(2.0)),
            rectangle2d.getCenter().getMeasureY().minus(rectangle2d.getMeasureYWidth().div(2.0))));
    corners.add(
        new Translation2d(
            rectangle2d.getCenter().getMeasureX().minus(rectangle2d.getMeasureXWidth().div(2.0)),
            rectangle2d.getCenter().getMeasureY().plus(rectangle2d.getMeasureYWidth().div(2.0))));
    return corners.toArray(new Translation2d[0]);
  }
}
