package frc.team4201.lib.simulation;

import static java.util.Arrays.stream;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.trajectory.Trajectory;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.ProtobufPublisher;
import org.wpilib.framework.RobotBase;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.SubsystemBase;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Class to handle all updates to the Field2D widget */
public class FieldSim extends SubsystemBase implements AutoCloseable {

  /** Enum of common {@link Field2d} objects */
  public enum FIELD_OBJECTS {
    /** robotPose */
    robotPose,
    /** modulePoses */
    modulePoses;

    /**
     * Return keys as List of Strings
     *
     * @return List of Strings
     */
    public static List<String> keysAsStrings() {
      return stream(FIELD_OBJECTS.values()).map(Enum::toString).toList();
    }
  };

  private final Field2d m_field2D = new Field2d();

  private final Map<String, Pose2d[]> m_objectPoses = new HashMap<>();

  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final NetworkTable field2dTable = nt.getTable("SmartDashboard").getSubTable("Field2D");
  private final ProtobufPublisher<Trajectory> trajectoryProtoPublisher =
      field2dTable.getProtobufTopic("trajectoryProto", Trajectory.proto).publish();

  /** Create a FieldSim object */
  public FieldSim() {
    SmartDashboard.putData("Field2D", m_field2D);
  }

  public Field2d getField2d() {
    return m_field2D;
  }

  public void initializeTranslations(String key, Translation2d... translations) {
    m_field2D
        .getObject(key)
        .setPoses(
            Arrays.stream(translations)
                .map(t -> new Pose2d(t, Rotation2d.kZero))
                .toArray(Pose2d[]::new));
  }

  /**
   * Add poses once to FieldSim.
   *
   * @param key The name of the object (Must be unique)
   * @param poses The poses corresponding to the object's position
   */
  public void initializePoses(String key, List<Pose2d> poses) {
    m_field2D.getObject(key).setPoses(poses);
  }

  /**
   * Add poses once to FieldSim.
   *
   * @param key The name of the object (Must be unique)
   * @param poses The poses corresponding to the object's position
   */
  public void initializePoses(String key, Pose2d... poses) {
    m_field2D.getObject(key).setPoses(poses);
  }

  public void addTranslations(String key, Translation2d... translations) {
    addPoses(
        key,
        Arrays.stream(translations)
            .map(t -> new Pose2d(t, Rotation2d.kZero))
            .toArray(Pose2d[]::new));
  }

  /**
   * Add a pose to FieldSim that will get updated.
   *
   * @param key The name of the object (Must be unique)
   * @param poses The poses corresponding to the object's position
   */
  public void addPoses(String key, Pose2d... poses) {
    m_objectPoses.put(key, poses);
    m_field2D.getObject(key).setPoses(poses);
  }

  /**
   * Remove a pose from {@link Field2d}. If it was added with addPoses(), also stop it from being
   * updated.
   *
   * @param key The name of the object
   */
  public void clearPose(String key) {
    m_objectPoses.remove(key);
    m_field2D.getObject(key).close();
  }

  /** Remove all poses from being displayed on FieldSim */
  public void clearAllPoses() {
    for (var entry : m_objectPoses.entrySet()) m_field2D.getObject(entry.getKey()).close();
    m_objectPoses.clear();
  }

  /**
   * Add a trajectory to be displayed in the Field2D widget.
   *
   * @param trajectory The WPILib Trajectory to display
   */
  public void addTrajectory(Trajectory trajectory) {
    m_field2D.getObject("trajectory").setTrajectory(trajectory);
    trajectoryProtoPublisher.accept(trajectory);
  }

  private void updateField2d() {
    if (m_objectPoses.containsKey("robotPose"))
      m_field2D.setRobotPose(m_objectPoses.get("robotPose")[0]);

    for (var entry : m_objectPoses.entrySet()) {
      if (FIELD_OBJECTS.keysAsStrings().contains(entry.getKey())) continue;

      m_field2D.getObject(entry.getKey()).setPoses(entry.getValue());
    }

    if (RobotBase.isSimulation()) {
      if (m_objectPoses.containsKey("modulePoses"))
        m_field2D.getObject("Swerve Modules").setPoses(m_objectPoses.get("modulePoses"));
    }
  }

  @Override
  public void periodic() {
    updateField2d();
  }

  @Override
  public void simulationPeriodic() {}

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
