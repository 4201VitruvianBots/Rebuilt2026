package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.generated.TunerConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import utils.TestUtils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class TestVision {
    CommandSwerveDrivetrain swerveDrive;
    Controls controls;
    Vision vision;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);

        swerveDrive = TunerConstants.createDrivetrain();
        controls = new Controls();
        vision = new Vision(controls);
        vision.registerSwerveDrive(swerveDrive);
    }

    @Disabled("Issue with JNI crash")
    public void test_isOnTarget() {
        // Set a fake target. For ease, this is (3,4) for a right-triangle
        Pose2d[] mockRobotToTarget = {Pose2d.kZero, new Pose2d(3, 4, Rotation2d.kZero)};
        TestUtils.setPrivateField(vision, "robotToTarget", mockRobotToTarget);

        // Set the Robot's Rotation. m_cachedState is an inherited value from CTRE's class that contains the values we want from swerveDrive.getState()
//        TestUtils.setPrivateField(swerveDrive, "m_cachedState", new SwerveDriveState());
        SwerveDriveState swerveDriveState = (SwerveDriveState) TestUtils.getPrivateObject(swerveDrive, "m_cachedState");
        SwerveJNI jni = (SwerveJNI) TestUtils.getPrivateObject(swerveDrive, "m_jni");
        jni.driveState = new SwerveJNI.DriveState();
        jni.driveState.PoseTheta = Degrees.of(52).in(Radians);
//        SwerveDriveState swerveDriveState = new SwerveDriveState();
//        jni.ModuleStates = new SwerveModuleState[] {
//                new SwerveModuleState(),
//                new SwerveModuleState(),
//                new SwerveModuleState(),
//                new SwerveModuleState()
//        };
//        jni.ModuleTargets = new SwerveModuleState[] {
//                new SwerveModuleState(),
//                new SwerveModuleState(),
//                new SwerveModuleState(),
//                new SwerveModuleState()
//        };
//        jni.ModulePositions = new SwerveModulePosition[] {
//                new SwerveModulePosition(),
//                new SwerveModulePosition(),
//                new SwerveModulePosition(),
//                new SwerveModulePosition()
//        };
//        jni.Pose = new Pose2d(0, 0, Rotation2d.fromDegrees(52));
//        TestUtils.setPrivateField(swerveDrive, "m_cachedState", swerveDriveState);

        Pose2d[] targetAngle = (Pose2d[]) TestUtils.getPrivateObject(vision, "robotToTarget");
        System.out.println("Calculated Angle Delta: " + vision.getTranslationDelta().in(Degrees));
        System.out.println("Robot Angle: " + swerveDrive.getState().Pose.getRotation().getDegrees());

        assertTrue(vision.isOnTarget());
    }

    @Test
    public void test_isOnTarget2() {
        Pose2d mockRobotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(53));
        Pose2d mockTargetPosition = new Pose2d(3, 4, Rotation2d.kZero);

        // Update this function
        var angleDelta = mockRobotPose.getRotation().minus(mockRobotPose.getRotation());

        System.out.println("Calculated Angle Delta: " + angleDelta.getDegrees());
        assertTrue(angleDelta.getMeasure().lt(Degrees.of(1)));
    }
}
