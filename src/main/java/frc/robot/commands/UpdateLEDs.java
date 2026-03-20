// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LED.LED_STATES;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Uptake;
import frc.team4201.lib.utils.HubTracker;

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
     * When robot enables, default to IDLE state, IDLE_CAN_ERROR if a CAN device is disconnected
     * During the last 3 seconds of a shift, display lights similar to the 
     * If the intake is running, set to INTAKING state
     * If the flywheel is running to shoot, set to SHOOTING state
     */
    final boolean shiftEnd = HubTracker.timeRemainingInCurrentShift()
        .map(timeRemain -> timeRemain.lt(Seconds.of(3.0)))
        .orElse(false);
    if (shiftEnd) {
        if ((HubTracker.isActive(Alliance.Red) && !HubTracker.isActiveNext(Alliance.Red))) {
            m_led.setState(LED_STATES.RED_SHIFT_END);
        } else if ((HubTracker.isActive(Alliance.Blue) && !HubTracker.isActiveNext(Alliance.Blue))) {
            m_led.setState(LED_STATES.BLUE_SHIFT_END);
        }
    } else if (m_flywheel.getRPMSetpoint() > 0) {
      m_led.setState(LED_STATES.SHOOTING, () -> (m_flywheel.getAbsoluteRPMerror() / m_flywheel.getRPMSetpoint()));
    } else if (MathUtil.applyDeadband(Math.abs(m_intake.getPercentOutput()), 0.05) != 0.0) {
      m_led.setState(LED_STATES.INTAKING);
    } else if (DriverStation.isEnabled()) {
      m_led.setState(LED_STATES.IDLE);
    } else {
      m_led.setState(LED_STATES.DISABLED);
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
