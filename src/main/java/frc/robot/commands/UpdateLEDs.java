// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Uptake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpdateLEDs extends Command {
  private final LEDs m_led;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Intake m_intake; // Used to track intaking state
  /* private final Climber m_climber; // Used to track climbing state */
  private final Uptake m_uptake; // Used to track shooting state

  /** Creates a new UpdateLEDs. */
  public UpdateLEDs(LEDs led, CommandSwerveDrivetrain drivetrain, Intake intake,
      /* Climber climber, */ Uptake uptake) {
    m_led = led;
    m_drivetrain = drivetrain;
    m_intake = intake;
    /* m_climber = climber; */
    m_uptake = uptake;
    
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
     * If the drivetrain is moving above a certain threshold, set to DRIVING state
     * If the intake is running, set to INTAKING state
     * If the uptake is running to shoot, set to SHOOTING state
     * If the climber is climbing, set to CLIMBING state. Climbing state is permanent until disabled or when switching from auto to teleop.
     */
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
