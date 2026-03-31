package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;

import java.util.function.DoubleSupplier;

public class ResetGyroWithAngle extends Command {
  /** Creates a new ResetGyro. */
  private final CommandSwerveDrivetrain m_swerveDrive;
  private final DoubleSupplier m_angleSupplier;

  public ResetGyroWithAngle(CommandSwerveDrivetrain swerveDrive, DoubleSupplier angleSupplier) {
    m_swerveDrive = swerveDrive;
    m_angleSupplier = angleSupplier;
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
    m_swerveDrive.resetGyro(m_angleSupplier.getAsDouble());
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
