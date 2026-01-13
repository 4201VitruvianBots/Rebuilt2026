// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4201.lib.utils;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import java.util.ArrayList;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import frc.team4201.lib.command.SwerveSubsystem;

/**
 * Utility class for working with autonomous trajectory following. Is currently designed around <a
 * href="https://github.com/mjansen4857/pathplanner">PathPlanner</a>.
 */
public class TrajectoryUtils {
  private final SwerveSubsystem m_swerveDrive;
  private final TrajectoryUtilsConfig m_config;

  /**
   * Construct a {@link TrajectoryUtils} object to manage trajectory following.
   *
   * @param swerveDrive The {@link SwerveSubsystem}
   */
  public TrajectoryUtils(SwerveSubsystem swerveDrive) {
    this(swerveDrive, new TrajectoryUtilsConfig());
  }

  /**
   * Construct a {@link TrajectoryUtils} object to manage trajectory following.
   *
   * @param swerveDrive The {@link SwerveSubsystem}
   * @param config The {@link TrajectoryUtilsConfig} to use
   */
  public TrajectoryUtils(SwerveSubsystem swerveDrive, TrajectoryUtilsConfig config) {
    m_swerveDrive = swerveDrive;
    m_config = config;
  }

  /**
   * Generate a PathPlanner Command to follow a named PathPlanner trajectory file.
   *
   * @param pathName The name of the PathPlanner Trajectory file to reference.
   * @return Command
   */
  public Command generatePPHolonomicCommand(String pathName) {
    try {
      return generatePPHolonomicCommand(
          PathPlannerPath.fromPathFile(pathName), this::flipPathByAlliance);
    } catch (Exception e) {
      DriverStation.reportError(
          String.format("Could not load PathPlanner Path '%s'", pathName), e.getStackTrace());
      return new WaitCommand(0);
    }
  }

  /**
   * Generate a PathPlanner Command to follow a named PathPlanner trajectory file.
   *
   * @param pathName The name of the PathPlanner Trajectory file to reference.
   * @param flipPath Option to flip the trajectory instead of using the robot's current {@link
   *     DriverStation.Alliance} color
   * @return Command
   */
  public Command generatePPHolonomicCommand(String pathName, BooleanSupplier flipPath) {
    try {
      return generatePPHolonomicCommand(PathPlannerPath.fromPathFile(pathName), flipPath);
    } catch (Exception e) {
      DriverStation.reportError(
          String.format("Could not load PathPlanner Path '%s'", pathName), e.getStackTrace());
      return new WaitCommand(0);
    }
  }

  /**
   * Generate a PathPlanner Command to follow a named PathPlanner trajectory file.
   *
   * @param path The {@link PathPlannerPath} to follow.
   * @return Command
   */
  public Command generatePPHolonomicCommand(PathPlannerPath path) {
    return generatePPHolonomicCommand(path, this::flipPathByAlliance);
  }

  /**
   * Generate a PathPlanner Command to follow a named PathPlanner trajectory file.
   *
   * @param path The {@link PathPlannerPath} to follow.
   * @param flipPath Option to flip the trajectory around the field's center.
   * @return Command
   */
  public Command generatePPHolonomicCommand(PathPlannerPath path, BooleanSupplier flipPath) {
    return resetRobotPoseAuto(path, flipPath)
        .andThen(
            new FollowPathCommand(
                    path,
                    () -> m_swerveDrive.getState().Pose, 
                    () -> m_swerveDrive.getState().Speeds,
                    m_swerveDrive::setChassisSpeedsAuto,
                    new PPHolonomicDriveController(
                        m_swerveDrive.getAutoTranslationPIDConstants(),
                        m_swerveDrive.getAutoRotationPIDConstants()),
                    m_swerveDrive.getAutoRobotConfig(),
                    flipPath,
                    m_swerveDrive)
                .withName(Objects.equals(path.name, "") ? "FollowPathCommand" : path.name));
  }

  /**
   * Reset the robot's initial pose for auto.
   *
   * @param pathName The name of the {@link PathPlannerPath} containing the initial {@link Pose2d}.
   * @return Command
   */
  private Command resetRobotPoseAuto(String pathName) {
    return resetRobotPoseAuto(pathName, this::flipPathByAlliance);
  }

  /**
   * Reset the robot's initial pose for auto.
   *
   * @param pathName The name of the {@link PathPlannerPath} containing the initial {@link Pose2d}.
   * @param flipPose {@link BooleanSupplier} to determine if the starting pose should be flipped.
   *     Default to using flipPathByAlliance.
   * @return Command
   */
  private Command resetRobotPoseAuto(String pathName, BooleanSupplier flipPose) {
    try {
      return resetRobotPoseAuto(PathPlannerPath.fromPathFile(pathName), flipPose);
    } catch (Exception e) {
      DriverStation.reportError(
          String.format("Could not load PathPlanner Path '%s'", pathName), e.getStackTrace());
      return new WaitCommand(0);
    }
  }

  /**
   * Reset the robot's initial pose for auto.
   *
   * @param path The {@link PathPlannerPath} containing the initial {@link Pose2d}.
   * @return Command
   */
  public Command resetRobotPoseAuto(PathPlannerPath path) {
    return resetRobotPoseAuto(path, this::flipPathByAlliance);
  }

  /**
   * Reset the robot's initial pose for auto.
   *
   * @param path The {@link PathPlannerPath} containing the initial {@link Pose2d}.
   * @param flipPose {@link BooleanSupplier} to determine if the starting pose should be flipped.
   *     Default to using flipPathByAlliance.
   * @return Command
   */
  private Command resetRobotPoseAuto(PathPlannerPath path, BooleanSupplier flipPose) {
    var ppPose = path.getStartingHolonomicPose();
    if (m_config.resetPoseOnAuto) {
      if (ppPose.isPresent()) {
        return resetRobotPoseAuto(ppPose.get(), flipPose);
      } else {
        throw new RuntimeException(
            "[TrajectoryUtils::resetRobotPoseAuto] PathPlannerPath does not have a starting holonomic pose!");
      }
    } else {
      return new WaitCommand(0);
    }
  }

  /**
   * Reset the robot's initial pose for auto.
   *
   * @param pose The {@link Pose2d} to use.
   * @return Command
   */
  private Command resetRobotPoseAuto(Pose2d pose) {
    return resetRobotPoseAuto(pose, this::flipPathByAlliance);
  }

  /**
   * Reset the robot's initial pose for auto.
   *
   * @param pose The {@link Pose2d} to use.
   * @param flipPose {@link BooleanSupplier} to determine if the starting pose should be flipped.
   *     Default to using flipPathByAlliance.
   * @return Command
   */
  private Command resetRobotPoseAuto(Pose2d pose, BooleanSupplier flipPose) {
    return new ConditionalCommand(
            new InstantCommand(
                () -> m_swerveDrive.resetPose(FlippingUtil.flipFieldPose(pose)), m_swerveDrive),
            new InstantCommand(() -> m_swerveDrive.resetPose(pose), m_swerveDrive),
            flipPose)
        .ignoringDisable(true)
        .withName("ResetRobotPoseAuto");
  }

  /**
   * Generate a WPILib {@link Trajectory} from PathPlanner's {@link PathPlannerPath}. NOTE: This
   * constructor should only be used for generating Trajectories used for displays.
   *
   * @param paths the {@link PathPlannerPath}(s) to use
   * @return {@link Trajectory}
   */
  public Trajectory getTrajectoryFromPathPlanner(PathPlannerPath... paths) {
    return getTrajectoryFromPathPlanner(this::flipPathByAlliance, paths);
  }

  /**
   * Generate a WPILib {@link Trajectory} from PathPlanner's {@link PathPlannerPath}
   *
   * @param flipPath Whether to flip the {@link PathPlannerPath} before displaying it.
   * @param paths the {@link PathPlannerPath}(s) to use
   * @return {@link Trajectory}
   */
  public Trajectory getTrajectoryFromPathPlanner(
      BooleanSupplier flipPath, PathPlannerPath... paths) {
    return getTrajectoryFromPathPlanner(flipPath, new TrajectoryConfig(1, 1), paths);
  }

  /**
   * Generate a WPILib {@link Trajectory} from PathPlanner's {@link PathPlannerPath}
   *
   * @param flipPath Whether to flip the {@link PathPlannerPath} before displaying it.
   * @param config The WPILib {@link TrajectoryConfig}.
   * @param paths the {@link PathPlannerPath}(s) to use
   * @return {@link Trajectory}
   */
  public Trajectory getTrajectoryFromPathPlanner(
      BooleanSupplier flipPath, TrajectoryConfig config, PathPlannerPath... paths) {
    var pathPoses = new ArrayList<Pose2d>();

    for (var path : paths) {
      if (flipPath.getAsBoolean()) {
        pathPoses.addAll(
            path.flipPath().getPathPoses().stream()
                .map(p -> new Pose2d(p.getTranslation(), p.getRotation().minus(Rotation2d.k180deg)))
                .toList());
      } else {
        pathPoses.addAll(path.getPathPoses());
      }
    }

    return TrajectoryGenerator.generateTrajectory(pathPoses, config);
  }

  private boolean flipPathByAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }

  /** Class for setting {@link TrajectoryUtils} settings */
  public static class TrajectoryUtilsConfig {
    /**
     * Setting to reset the robot's pose at the beginning of auto to match the PathPlanner
     * trajectory
     */
    public boolean resetPoseOnAuto = true;

    /** Create a new instance of {@link TrajectoryUtilsConfig} */
    public TrajectoryUtilsConfig() {}

    /**
     * Setting to reset the robot's pose at the beginning of auto to match the PathPlanner
     * trajectory
     *
     * @param enable To enable this.
     * @return Itself
     */
    public TrajectoryUtilsConfig withResetPoseOnAuto(boolean enable) {
      this.resetPoseOnAuto = enable;
      return this;
    }
  }
}
