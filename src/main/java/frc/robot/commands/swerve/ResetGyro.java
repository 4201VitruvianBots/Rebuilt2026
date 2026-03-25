package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;

public class ResetGyro extends Command {
  /** Creates a new ResetGyro. */
  private final CommandSwerveDrivetrain m_swerveDrive;

  public ResetGyro(CommandSwerveDrivetrain swerveDrive) {
    m_swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_swerveDrive);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Controls.isRedAlliance()) {
      m_swerveDrive.resetGyro(180);
    } else {
      m_swerveDrive.resetGyro(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
