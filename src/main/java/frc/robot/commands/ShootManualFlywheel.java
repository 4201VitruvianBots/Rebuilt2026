package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class ShootManualFlywheel extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Flywheel m_flywheel;

  public ShootManualFlywheel(Flywheel flywheel) {
    m_flywheel = flywheel;

    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.setManualRPMOutputFOC(m_flywheel.getRPMSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.setTorqueCurrentOutputFOC(Volts.of(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
