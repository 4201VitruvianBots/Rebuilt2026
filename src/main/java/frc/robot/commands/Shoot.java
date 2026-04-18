package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FLYWHEEL;
import frc.robot.subsystems.*;

public class Shoot extends Command {
  private final Flywheel m_flywheel;
  private CommandXboxController m_driverController;
  private final Hood m_shooterHood;

  /** Shoot on the move command with rumble */
  public Shoot(
      Flywheel flywheel,
      Hood shooterHood,
      Uptake uptake,
      CommandXboxController driverController) {
    m_flywheel = flywheel;
    m_driverController = driverController;
    m_shooterHood = shooterHood;

    addRequirements(flywheel, shooterHood);
    SmartDashboard.putData(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.setRPMOutputFOC(RPM.of(800)); // Constant values until dynamic controls are added
    m_shooterHood.setAngle(Degrees.of(5));
    if (m_flywheel.isAtRPMsetpoint()) {
      m_driverController.setRumble(RumbleType.kBothRumble, FLYWHEEL.kRumbleStrength);
    } else {
      m_driverController.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driverController.setRumble(RumbleType.kBothRumble, 0);
    m_flywheel.setTorqueCurrentOutputFOC(Volts.of(0.0));
    m_flywheel.setRPMOutputFOC(RPM.of(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
