// package frc.team4201.lib.utils;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import frc.robot.LimelightHelpers;
// import frc.robot.constants.FIELD;
// import frc.robot.constants.VISION;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import java.util.Optional;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;

// /** Use Photonlib to simulate Limelights */
// public class LimelightSim {
//   CommandSwerveDrivetrain m_swerveDrive;

//   private PhotonPoseEstimator photonEstimatorLLr;
//   private PhotonPoseEstimator photonEstimatorLLl;
//   private PhotonCamera limelightR;
//   private PhotonCamera limelightL;

//   private VisionSystemSim visionSim;

//   private PhotonCameraSim llrSim;
//   private PhotonCameraSim lllSim;

//   public LimelightSim(CommandSwerveDrivetrain swerveDrive) {
//     m_swerveDrive = swerveDrive;

//     limelightR = new PhotonCamera(VISION.CAMERA_SERVER.limelightR.toString());
//     limelightL = new PhotonCamera(VISION.CAMERA_SERVER.limelightL.toString());
//     photonEstimatorLLr =
//         new PhotonPoseEstimator(
//             FIELD.aprilTagFieldLayout,
//             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//             VISION.limelightFPosition);
//     photonEstimatorLLr.setMultiTagFallbackStrategy(
//         PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
//     photonEstimatorLLl =
//         new PhotonPoseEstimator(
//             FIELD.aprilTagFieldLayout,
//             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//             VISION.limelightFPosition);
//     photonEstimatorLLl.setMultiTagFallbackStrategy(
//         PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

//     visionSim = new VisionSystemSim("limelightSim");
//     visionSim.addAprilTags(FIELD.aprilTagFieldLayout);

//     var cameraProp = new SimCameraProperties();
//     cameraProp.setCalibration(1280, 960, new Rotation2d(VISION.kLimelight4DFOV));
//     cameraProp.setFPS(45);
//     // TODO: Check these values
//     cameraProp.setCalibError(0.35, 0.10);
//     cameraProp.setAvgLatencyMs(50);
//     cameraProp.setLatencyStdDevMs(15);
//     llrSim = new PhotonCameraSim(limelightR, cameraProp);
//     lllSim = new PhotonCameraSim(limelightL, cameraProp);
//     visionSim.addCamera(llrSim, VISION.limelightFPosition);
//     visionSim.addCamera(lllSim, VISION.limelightBPosition);

//     llrSim.enableDrawWireframe(true);
//     lllSim.enableDrawWireframe(true);
//   }

//   public LimelightHelpers.PoseEstimate getSimulatedResultsLimelightF() {
//     //        LimelightHelpers.PoseEstimate results = null;
//     LimelightHelpers.PoseEstimate results = new LimelightHelpers.PoseEstimate();
//     Optional<EstimatedRobotPose> visionEst = Optional.empty();
//     for (var change : limelightR.getAllUnreadResults()) {
//       visionEst = photonEstimatorLLr.update(change);
//       change.metadata.getLatencyMillis();

//       if (RobotBase.isSimulation()) {
//         visionEst.ifPresentOrElse(
//             est ->
//                 getSimDebugField()
//                     .getObject("VisionEstimationLLr")
//                     .setPose(est.estimatedPose.toPose2d()),
//             () -> {
//               getSimDebugField().getObject("VisionEstimationLLr").setPoses();
//             });
//       }
//     }
//     // TODO: WIP
//     //        visionEst.ifPresent(p->{
//     //            results = new LimelightHelpers.PoseEstimate(
//     //                p.estimatedPose.toPose2d(),
//     //                p.timestampSeconds,
//     //                p.
//     //            );
//     //            results.botpose_wpiblue = new double[]{
//     //                    p.estimatedPose.getX(),
//     //                    p.estimatedPose.getY(),
//     //                    p.estimatedPose.getZ(),
//     //                    p.estimatedPose.getRotation().getMeasureX().in(Degrees),
//     //                    p.estimatedPose.getRotation().getMeasureY().in(Degrees),
//     //                    p.estimatedPose.getRotation().getMeasureZ().in(Degrees),
//     //            };
//     //            results.
//     //            if(DriverStation.)
//     //        });
//     //
//     return results;
//   }

//   public void simulationPeriodic() {
//     visionSim.update(m_swerveDrive.getState().Pose);
//   }

//   public void resetSimPose(Pose2d pose) {
//     if (RobotBase.isSimulation()) visionSim.resetRobotPose(pose);
//   }

//   public Field2d getSimDebugField() {
//     if (!RobotBase.isSimulation()) return null;
//     return visionSim.getDebugField();
//   }
// }
