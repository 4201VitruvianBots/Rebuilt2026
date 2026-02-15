// // // Copyright (c) FIRST and other WPILib contributors.
// // // Open Source Software; you can modify and/or share it under the terms of
// // // the WPILib BSD license file in the root directory of this project.

// // package frc.robot.commands.autos;

// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.path.PathPlannerPath;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import frc.robot.commands.IntakeCommand;
// import frc.robot.commands.Shoot;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Flywheel;
// import frc.robot.subsystems.Hood;
// import frc.robot.subsystems.Indexer;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.IntakePivot;
// import frc.robot.subsystems.Uptake;
// import frc.robot.subsystems.Vision;
// import frc.team4201.lib.command.Auto;
// import frc.team4201.lib.utils.TrajectoryUtils;
// import java.util.function.BooleanSupplier;

// public class PreloadNeutralShootClimb extends Auto {
//   public PreloadNeutralShootClimb(
//       CommandSwerveDrivetrain swerveDrive,
//       Intake intake,
//       Vision vision,
//       Flywheel flywheel,
//       Hood hood,
//       IntakePivot intakePivot,
//       Indexer indexer,
//       Uptake uptake,
//       BooleanSupplier flipPath) {
//     try {
//       var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

// //       TrajectoryUtils trajectoryUtils = swerveDrive.getTrajectoryUtils();

// //       var m_path1 = PathPlannerPath.fromPathFile("PreloadNeutralShootClimb1");
// //       var m_path2 = PathPlannerPath.fromPathFile("PreloadNeutralShootClimb2");
// //       var m_path3 = PathPlannerPath.fromPathFile("PreloadNeutralShootClimb3");
// //       var m_path4 = PathPlannerPath.fromPathFile("PreloadNeutralShootClimb4");
// //       var m_path5Outpost = PathPlannerPath.fromPathFile("PreloadNeutralShootClimb5Outpost");
// //       var m_path5Depot = PathPlannerPath.fromPathFile("PreloadNeutralShootClimb5Depot");

//       addCommands(
//           getPathCommand(trajectoryUtils, m_path1, flipPath)
//               .andThen(() -> swerveDrive.setControl(stopRequest)),
//           new Shoot(flywheel, vision, hood).withTimeout(3),
//           getPathCommand(trajectoryUtils, m_path2, flipPath)
//               .andThen(() -> swerveDrive.setControl(stopRequest)),
//           new ParallelRaceGroup(
//               new IntakeCommand(intake, intakePivot, indexer, uptake),
//               getPathCommand(trajectoryUtils, m_path3, flipPath)
//                   .andThen(() -> swerveDrive.setControl(stopRequest))),
//           getPathCommand(trajectoryUtils, m_path4, flipPath)
//               .andThen(() -> swerveDrive.setControl(stopRequest)),
//           new Shoot(flywheel, vision, hood).withTimeout(3),
//           getChoiceCommand(trajectoryUtils, m_path5Outpost, m_path5Depot, flipPath)
//               .andThen(() -> swerveDrive.setControl(stopRequest))
//           // Todo: add climb (command not yet implemented in this branch)
//           );
//     } catch (Exception e) {
//       DriverStation.reportError(
//           "Failed to load path for PreloadNeutralShootClimb", e.getStackTrace());
//       addCommands(new InstantCommand());
//     }
//   }
// }
