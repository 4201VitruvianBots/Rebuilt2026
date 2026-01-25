// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.INTAKE.ROLLERS.INTAKE_SPEED;
import frc.robot.Constants.SHOOTER.HOOD.HOOD_ANGLE;
import frc.robot.Constants.SHOOTER.FLYWHEEL.SHOOTER_VELOCITY;
import frc.robot.Constants.SWERVE;
import frc.robot.Constants.UPTAKE.UPTAKE_SPEED;
import frc.robot.Constants.USB;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.RunUptake;
import frc.robot.commands.Shoot;
import frc.robot.commands.UpdateLEDs;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.Uptake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged(name = "RobotContainer", importance = Logged.Importance.CRITICAL)
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  @Logged(name = "ShooterFlywheel", importance = Logged.Importance.INFO)
  private ShooterFlywheel m_shooterFlywheel;

  @Logged(name = "ShooterHood", importance = Logged.Importance.INFO)
  private ShooterHood m_shooterHood;

  @Logged(name = "Indexer", importance = Logged.Importance.INFO)
  private Indexer m_Indexer;

  @Logged(name = "Intake", importance = Logged.Importance.INFO)
  private Intake m_Intake = new Intake();

  @Logged(name = "Uptake", importance = Logged.Importance.INFO)
  private Uptake m_Uptake = new Uptake();

  @Logged(name = "LEDs", importance = Logged.Importance.INFO)
  private LEDs m_led = new LEDs();

  private final CommandSwerveDrivetrain m_swerveDrive = TunerConstants.createDrivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(USB.driver_xBoxController);

  @NotLogged
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Kspeed at 12 volts desired top speed

  @NotLogged
  private double MaxAngularRate =
      RotationsPerSecond.of(SWERVE.kMaxRotationRadiansPerSecond)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1); // Add a 10% deadband

  @Logged(name = "AutoChooser")
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    initializeSubSystems();
    configureBindings();
    initSmartDashboard();
  }

  private void initializeSubSystems() {
    m_shooterFlywheel = new ShooterFlywheel();
    m_shooterHood = new ShooterHood();
    m_Indexer = new Indexer();
    m_Intake = new Intake();
    m_Uptake = new Uptake();
    m_swerveDrive.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_swerveDrive.applyRequest(
            () -> {
              var rotationRate = -m_driverController.getRightX() * MaxAngularRate;
              // // if heading target
              // if (m_swerveDrive.isTrackingState()) {
              //   rotationRate = m_swerveDrive.calculateRotationToTarget();
              // }
              drive
                  .withVelocityX(
                      -m_driverController.getLeftY()
                          * MaxSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(
                      -m_driverController.getLeftX()
                          * MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(
                      rotationRate); // Drive counterclockwise with negative X (left)
              return drive;
            }));
    m_led.setDefaultCommand(new UpdateLEDs(m_led, m_swerveDrive, m_Intake, /* m_Climber, */ m_Uptake));
  }

  private void configureBindings() {
    if (m_shooterFlywheel != null && m_shooterHood != null) {
      m_driverController
          .a()
          .whileTrue(
              new Shoot(
                  m_shooterFlywheel,
                  m_shooterHood,
                  SHOOTER_VELOCITY.HIGH,
                  HOOD_ANGLE.CLOSE.getAngle()));
    }
    // m_driverController.a().whileTrue(new Shoot(m_ShooterFlywheel, ShooterRPM.HIGH.getRPM()));
    // m_driverController.b().whileTrue(new Index(m_Indexer, INDEXERSPEED.INDEXING));
    m_driverController.leftBumper().whileTrue(new RunIntake(m_Intake, INTAKE_SPEED.INTAKING));
    m_driverController.rightBumper().whileTrue(new RunUptake(m_Uptake, UPTAKE_SPEED.UPTAKING));
  }

  private void initAutoChooser() {

    SmartDashboard.putData("Auto Mode", m_chooser);
    m_chooser.setDefaultOption("Do Nothing", new WaitCommand(0));
  }

  private void initSmartDashboard() {
    initAutoChooser();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand();
  }
}
