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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.INDEXER.INDEXER_SPEED;
import frc.robot.Constants.INTAKE.ROLLERS.INTAKE_SPEED;
import frc.robot.Constants.SWERVE;
import frc.robot.Constants.UPTAKE.UPTAKE_SPEED;
import frc.robot.Constants.USB;
import frc.robot.commands.AutoAlignDrive;
import frc.robot.commands.Index;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RunUptake;
import frc.robot.commands.UpdateLEDs;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootManualFlywheel;
import frc.robot.commands.autos.*;
import frc.robot.generated.WoodBotConstants;
import frc.robot.simulation.Robot2d;
import frc.robot.subsystems.*;
import frc.team4201.lib.simulation.FieldSim;
import frc.team4201.lib.utils.Telemetry;
import frc.team4201.lib.utils.HubTracker;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged(name = "RobotContainer", importance = Logged.Importance.CRITICAL)
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  @Logged(name = "Flywheel", importance = Logged.Importance.INFO)
  private Flywheel m_flywheel = new Flywheel();

  @Logged(name = "Hood", importance = Logged.Importance.INFO)
  private Hood m_hood = new Hood();

  private CommandSwerveDrivetrain m_swerveDrive = WoodBotConstants.createDrivetrain();

  @Logged(name = "Intake", importance = Logged.Importance.INFO)
  private Intake m_intake;

  private Controls m_controls = new Controls();

  @Logged(name = "Vision", importance = Logged.Importance.INFO)
  private Vision m_vision = new Vision(m_controls);

  @Logged(name = "Indexer", importance = Logged.Importance.INFO)
  private Indexer m_indexer = new Indexer();

  @Logged(name = "Uptake", importance = Logged.Importance.INFO)
  private Uptake m_uptake = new Uptake();

  @Logged(name = "Climber", importance = Logged.Importance.INFO)
  private Climber m_climber = new Climber();

  @Logged(name = "LEDs", importance = Logged.Importance.INFO)
  private LEDs m_led = new LEDs();

  @Logged(name = "IntakePivot", importance = Logged.Importance.INFO)
  private IntakePivot m_intakePivot = new IntakePivot();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(USB.driver_xBoxController);

  @Logged(name = "IsHubActive", importance = Logged.Importance.CRITICAL)
  public boolean isHubActive() {
    return HubTracker.isAllianceHubActive();
  }

  @NotLogged
  private double MaxSpeed =
      WoodBotConstants.kSpeedAt12Volts.in(MetersPerSecond); // Kspeed at 12 volts desired top speed

  private Boolean m_flipToRight = false;

  @NotLogged
  private double MaxAngularRate =
      RotationsPerSecond.of(SWERVE.kMaxRotationRadiansPerSecond)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1); // Add a 10% deadband

  private Robot2d m_robotSim;
  private final Telemetry m_telemetry = new Telemetry(MaxSpeed, SWERVE.kModuleTranslations);
  private final FieldSim m_fieldSim = new FieldSim();

  @Logged(name = "AutoChooser")
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private final SendableChooser<Boolean> m_autoSide = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    initializeSubSystems();
    configureBindings();
    initSmartDashboard();

    m_telemetry.registerFieldSim(m_fieldSim);
    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
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
    m_uptake = new Uptake();
    m_indexer = new Indexer();
    m_led.setDefaultCommand(new UpdateLEDs(m_led, m_swerveDrive, m_intake, m_climber, m_uptake));

    if (Robot.isSimulation()) {
      m_robotSim = new Robot2d();
      m_robotSim.registerSubsystems(
          m_flywheel, m_hood, m_indexer, m_intake, m_uptake);
    }
  }

  private void configureBindings() {
    // aim at target
    if (m_swerveDrive != null && m_vision != null && m_flywheel != null) {
      m_driverController
          .rightBumper()
          .toggleOnTrue(
              new ParallelCommandGroup(
                  new AutoAlignDrive(
                      m_swerveDrive,
                      m_vision,
                      () -> m_driverController.getLeftY(),
                      () -> m_driverController.getLeftX()),
                  new Shoot(m_flywheel, m_vision)));
    }

    if (m_swerveDrive != null && m_vision != null) {
      m_driverController
          .leftBumper()
          .toggleOnTrue(
              new AutoAlignDrive(
                  m_swerveDrive,
                  m_vision,
                  () -> m_driverController.getLeftY(),
                  () -> m_driverController.getLeftX()));
    }

    if (m_swerveDrive != null && m_flywheel != null && m_vision != null) {
      m_driverController.a().whileTrue(new Shoot(m_flywheel, m_vision));
    }

    if (m_flywheel != null) {
      m_driverController.y().whileTrue(new ShootManualFlywheel(m_flywheel));
    }

    // I forsee a state machine in the future...
    if (m_uptake != null && m_indexer != null && m_intake != null) {
      m_driverController
          .rightTrigger()
          .whileTrue(
              new ParallelCommandGroup(
                  new RunUptake(m_uptake, UPTAKE_SPEED.UPTAKING),
                  new Index(m_indexer, INDEXER_SPEED.INDEXING),
                  new RunIntake(m_intake, INTAKE_SPEED.INTAKING)));
    }

    if (m_intake != null) {
      m_driverController.leftTrigger().whileTrue(new RunIntake(m_intake, INTAKE_SPEED.INTAKING));
    }
  }

  private void initAutoChooser() {
    SmartDashboard.putData("Auto Mode", m_autoChooser);
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    m_autoChooser.addOption(
        "PreloadDepotShootMiddle",
        new PreloadDepotShootMiddle(m_swerveDrive, m_intake, m_vision, m_flywheel));
    m_autoChooser.addOption(
        "PreloadNeutralShootClimb",
        new PreloadNeutralShootClimb(
            m_swerveDrive, m_intake, m_vision, m_flywheel, () -> m_flipToRight));
    m_autoChooser.addOption(
        "PreloadNeutralDepotClimb",
        new PreloadNeutralDepotClimb(m_swerveDrive, m_intake, m_vision, m_flywheel));
    m_autoChooser.addOption(
        "PreloadNeutralShootTwice",
        new PreloadNeutralShootTwice(
            m_swerveDrive, m_intake, m_vision, m_flywheel, () -> m_flipToRight));
    m_autoChooser.addOption(
        "PreloadCenter", new PreloadCenter(m_swerveDrive, m_intake, m_vision, m_flywheel));
  }

  private void initSideChooser() {
    SmartDashboard.putData("Auto Side", m_autoSide);
    m_autoSide.setDefaultOption("No Flip", false);

    m_autoSide.addOption("Depot", false);
    m_autoSide.addOption("Outpost", true);
    m_autoSide.onChange(
        (Boolean selected) -> {
          m_flipToRight = selected;
        });
  }

  private void initSmartDashboard() {
    initAutoChooser();
    initSideChooser();
    SmartDashboard.putData("ResetGyro", new ResetGyro(m_swerveDrive));
  }

  public void testInit() {
    if (m_flywheel != null) m_flywheel.testInit();
    if (m_vision != null) m_vision.testInit();
  }

  public void testPeriodic() {
    if (m_flywheel != null) m_flywheel.testPeriodic();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }
}
