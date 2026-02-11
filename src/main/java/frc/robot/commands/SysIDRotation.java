package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SysIDRotation extends Command {
  private final CommandSwerveDrivetrain m_swerveDrivetrain;
  private final boolean m_isQuasiStatic;
  private final Direction m_direction;

  /** Creates a new AutoAlign. */
  public SysIDRotation(
      CommandSwerveDrivetrain commandSwerveDrivetrain, boolean isQuasiStatic, Direction direction) {
    m_swerveDrivetrain = commandSwerveDrivetrain;
    m_isQuasiStatic = isQuasiStatic;
    m_direction = direction;
    addRequirements(m_swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrivetrain.initDriveSysid();
    if (m_isQuasiStatic) {
      m_swerveDrivetrain.sysIdQuasistatic(m_direction);
    } else {
      m_swerveDrivetrain.sysIdDynamic(m_direction);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
