// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LED.LED_STATES;
import frc.robot.subsystems.LEDs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestLEDs extends Command {
  private final LEDs m_led;
  private Timer m_time_since_led_start = new Timer();

  /** Creates a new TestLEDs. */
  public TestLEDs(LEDs led) {
    m_led = led;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time_since_led_start.reset();
    m_time_since_led_start.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Every 2 seconds, cycle to the next LED state
    int state_index = (int) (m_time_since_led_start.get() / 2) % LED_STATES.values().length;
    m_led.setState(LED_STATES.values()[state_index], () -> (m_time_since_led_start.get() % 2 / 2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
