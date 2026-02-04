package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE.MOTOR_TYPE;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwitchSwerveNeutralModeState extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final CommandSwerveDrivetrain m_swerveDrive;
  private final NeutralModeValue m_neutralModeValue;

  public SwitchSwerveNeutralModeState(CommandSwerveDrivetrain swerveDrive, NeutralModeValue neutralModeValue) {
    m_swerveDrive = swerveDrive;
    m_neutralModeValue = neutralModeValue;
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    m_swerveDrive.setNeutralMode(MOTOR_TYPE.STEER, m_neutralModeValue); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
