// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LED.LED_STATES;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Uptake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpdateLEDs extends Command {
  private final LEDs m_led;
  private final Intake m_intake; // Used to track intaking state
  private final Flywheel m_flywheel; // Used to track shooting state

  /** Creates a new UpdateLEDs. */
  public UpdateLEDs(
      LEDs led, Intake intake, Flywheel flywheel) {
    m_led = led;
    m_intake = intake;
    m_flywheel = flywheel;

    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * When disabled, DISABLED state always takes priority
     * When robot enables, default to IDLE state
     * If the intake is running, set to INTAKING state
     * If the flywheel is running to shoot, set to SHOOTING state
     */
    if (m_flywheel.getRPMSetpoint() > 0) {
      m_led.setState(LED_STATES.SHOOTING, () -> (m_flywheel.getAbsoluteRPMerror() / m_flywheel.getRPMSetpoint()));
    } else if (MathUtil.applyDeadband(Math.abs(m_intake.getPercentOutput()), 0.05) != 0.0) {
      m_led.setState(LED_STATES.INTAKING, () -> 0.0);
    } else if (DriverStation.isEnabled()) {
      m_led.setState(LED_STATES.IDLE, () -> 0.0);
    } else {
      m_led.setState(LED_STATES.DISABLED, () -> 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean runsWhenDisabled() {
    return true;
  }
}
