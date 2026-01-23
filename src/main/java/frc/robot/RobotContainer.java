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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.INTAKEMOTORS.ROLLERS.INTAKESPEED;
import frc.robot.Constants.SHOOTERHOOD.ManualAngle;
import frc.robot.Constants.SHOOTERMOTORS.ManualRPS;
import frc.robot.Constants.SWERVE;
import frc.robot.Constants.USB;
import frc.robot.commands.AutoAlignDrive;
import frc.robot.commands.Index;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootFlywheel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterRollers;
import frc.robot.subsystems.Vision;
import frc.team4201.lib.simulation.FieldSim;
import frc.team4201.lib.utils.Telemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged(name = "RobotContainer", importance = Logged.Importance.CRITICAL)
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  @Logged(name = "ShooterRollers", importance = Logged.Importance.INFO)
  private ShooterRollers m_shooterRollers = new ShooterRollers();

  @Logged(name = "ShooterHood", importance = Logged.Importance.INFO)
  private ShooterHood m_shooterHood;

  private CommandSwerveDrivetrain m_swerveDrive = TunerConstants.createDrivetrain();
  @Logged(name = "Intake", importance = Logged.Importance.INFO)
  private Intake m_intake;

  private Controls m_controls = new Controls();

  @Logged(name = "Vision", importance = Logged.Importance.INFO)
  private Vision m_vision = new Vision(m_controls);

  private Telemetry m_telemetry = new Telemetry();

  private FieldSim m_fieldSim = new FieldSim();

  // private Indexer m_Indexer = new Indexer();
  // @Logged(name = "Indexer", importance = Logged.Importance.INFO)
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
    m_vision.registerSwerveDrive(m_swerveDrive);
    m_vision.registerFieldSim(m_fieldSim);
    m_telemetry.registerFieldSim(m_fieldSim);
    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
    m_intake = new Intake();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // aim at target
    m_driverController
        .rightBumper()
        .whileTrue(
            new AutoAlignDrive(
                m_swerveDrive,
                () -> m_driverController.getLeftY(),
                () -> m_driverController.getLeftX()));
    if (m_shooterRollers != null && m_shooterHood != null) {
      m_driverController
          .b()
          .whileTrue(
              new Shoot(
                  m_shooterRollers, m_shooterHood, ManualRPS.HIGH, ManualAngle.CLOSE.getAngle()));
    }

    if (m_shooterRollers != null) {
      m_driverController.a().whileTrue(new ShootFlywheel(m_shooterRollers, ManualRPS.HIGH));
    }

    if (m_intake != null) {
      m_driverController.rightBumper().whileTrue(new RunIntake(m_intake, INTAKESPEED.INTAKING));
    }

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
