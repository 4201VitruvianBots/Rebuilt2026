package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FIELD;
import frc.robot.constants.SWERVE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class AutoAlignDrive extends Command {
  private final CommandSwerveDrivetrain m_swerveDrivetrain;

  private final DoubleSupplier m_throttleInput;
  private final DoubleSupplier m_strafeInput;

  /** Creates a new AutoAlign. */
  public AutoAlignDrive(
      CommandSwerveDrivetrain commandSwerveDrivetrain,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput) {
    m_swerveDrivetrain = commandSwerveDrivetrain;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    addRequirements(m_swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Add pass position as goal
    // If we're outside our own zone, then we align to pass.
    //    if (m_vision.isInOpposingAllianceSector() || m_vision.isInNeutralSector()) {
    //      m_goal = Translation2d.kZero;
    //      // If we're in our own zone, then we align to the hub
    //    } else {
    //      m_goal = FIELD.HUB.GOAL.getTargetPosition().toTranslation2d();
    //    }
    var setPoint =
        FIELD
            .HUB
            .GOAL
            .getTargetPosition()
            .toTranslation2d()
            .minus(m_swerveDrivetrain.getState().Pose.getTranslation());

    m_swerveDrivetrain.setChassisSpeedsWithHeading(
        SWERVE.kMaxSpeed.times(m_throttleInput.getAsDouble()),
        SWERVE.kMaxSpeed.times(m_strafeInput.getAsDouble()),
        setPoint.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
