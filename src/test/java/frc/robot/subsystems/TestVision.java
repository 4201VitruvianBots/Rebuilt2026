package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.doReturn;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.TunerConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Spy;
import org.mockito.junit.jupiter.MockitoExtension;
import utils.TestUtils;

@ExtendWith(MockitoExtension.class)
public class TestVision {
  @Spy CommandSwerveDrivetrain swerveDrive = TunerConstants.createDrivetrain();
  Controls controls = new Controls();
  @Spy Vision vision = new Vision(controls);

  @BeforeEach
  void setup() {
    vision.registerSwerveDrive(swerveDrive);
  }

  @Test
  public void test_isOnTarget() {
    // Set a fake target. For ease, this is (3,4) for a right-triangle
    Pose2d[] mockRobotToTarget = {Pose2d.kZero, new Pose2d(3, 4, Rotation2d.kZero)};
    TestUtils.setPrivateField(vision, "robotToTarget", mockRobotToTarget);

    // Set the robot's position for this test
    SwerveDriveState swerveDriveState = new SwerveDriveState();
    swerveDriveState.Pose = new Pose2d(0, 0, Rotation2d.fromDegrees(53));
    // Use mockito to have the swerveDrive use our fake position
    doReturn(swerveDriveState).when(swerveDrive).getState();

    System.out.println("Robot Angle: " + swerveDrive.getState().Pose.getRotation().getDegrees());

    assertTrue(vision.isOnTarget());
  }
}
