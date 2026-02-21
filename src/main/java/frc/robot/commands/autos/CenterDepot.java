// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Shoot;
import frc.robot.constants.UPTAKE.UPTAKE_SPEED;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Vision;

public class CenterDepot extends SequentialCommandGroup {
  public CenterDepot(
      CommandSwerveDrivetrain swerveDrive,
      Intake intake,
      Vision vision,
      Flywheel flywheel,
      Hood hood,
      IntakePivot intakePivot,
      Indexer indexer,
      Uptake uptake) {
    try {
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var m_path1 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("CenterDepot1");
      var m_path2 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("CenterDepot2");
      var m_path3 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("CenterDepot3");
      addCommands(
          m_path1.andThen(() -> swerveDrive.setControl(stopRequest)),
          new IntakeCommand(intake, intakePivot, indexer, uptake).withTimeout(9),
          new ParallelCommandGroup(
                  m_path2.andThen(() -> swerveDrive.setControl(stopRequest)),
                  new Shoot(flywheel, hood, vision, swerveDrive), 
                  uptake.command(UPTAKE_SPEED.UPTAKING)).withTimeout(3),
          m_path3.andThen(() -> swerveDrive.setControl(stopRequest)));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for CenterDepot", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
