package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INTAKE.ROLLERS.INTAKE_SPEED;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {

  private final Intake m_intake;

  private final INTAKE_SPEED m_speed;

  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, INTAKE_SPEED speed) {
    m_intake = intake;
    m_speed = speed;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setOutputPercent(m_speed.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setOutputPercent(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
