// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.hammerheads5000.FuelSim;
import frc.robot.commands.Fire;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JostleIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.UpdateLEDs;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.autos.routines.CenterPreload;
import frc.robot.commands.autos.routines.JustDepot;
import frc.robot.commands.autos.routines.NextLevelAuto;
import frc.robot.commands.autos.routines.SimboticsAuto;
import frc.robot.commands.autos.routines.SingleScoopWithSprinkles;
import frc.robot.commands.autos.routines.TwoCycle;
import frc.robot.commands.autos.routines.TwoCycleInsideOutRush;
import frc.robot.commands.autos.routines.TwoCycleRush;
import frc.robot.commands.autos.segments.IntakeFromNeutral;
import frc.robot.commands.swerve.ResetGyro;
import frc.robot.constants.FIELD;
import frc.robot.constants.FLYWHEEL;
import frc.robot.constants.INTAKE.PIVOT.PIVOT_SETPOINT;
import frc.robot.constants.INTAKE.ROLLERS.INTAKE_STATE;
import frc.robot.constants.ROBOT;
import frc.robot.constants.ROBOT.ROBOT_ID;
import frc.robot.constants.ROBOT.SIM;
import frc.robot.constants.ROBOT.TWO_CYCLE_PATH;
import frc.robot.constants.ROBOT.USB;
import frc.robot.constants.SWERVE;
import frc.robot.constants.SWERVE.MOTOR_TYPE;
import frc.robot.generated.V1Constants;
import frc.robot.generated.V2Constants;
import frc.robot.simulation.Robot2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Vision;
import frc.team4201.lib.simulation.FieldSim;
import frc.team4201.lib.utils.HubTracker;
import frc.team4201.lib.utils.POVUtils;
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
  @Logged(name = "Flywheel", importance = Logged.Importance.INFO)
  private Flywheel m_flywheel;

  @Logged(name = "Hood", importance = Logged.Importance.INFO)
  private Hood m_hood;

  private CommandSwerveDrivetrain m_swerveDrive;

  @Logged(name = "Intake", importance = Logged.Importance.INFO)
  private Intake m_intake;

  private Controls m_controls;

  @Logged(name = "Vision", importance = Logged.Importance.INFO)
  private Vision m_vision;

  @Logged(name = "Indexer", importance = Logged.Importance.INFO)
  private Indexer m_indexer;

  @Logged(name = "Uptake", importance = Logged.Importance.INFO)
  private Uptake m_uptake;

  @Logged(name = "LEDs", importance = Logged.Importance.INFO)
  private LEDs m_led;

  // @Logged(name = "IntakePivot", importance = Logged.Importance.INFO)
  @Logged(name = "IntakePivot", importance = Logged.Importance.INFO)
  private IntakePivot m_intakePivot;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(USB.driver_xBoxController);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(USB.operator_xboxController);

  @Logged(name = "IsHubActive", importance = Logged.Importance.CRITICAL)
  public boolean isHubActive() {
    return HubTracker.isActive();
  }

  @NotLogged
  private final LinearVelocity MaxSpeed =
      V2Constants.kSpeedAt12Volts; // kSpeed at 12 volts desired top speed

  @NotLogged
  private final AngularVelocity MaxAngularRate =
      SWERVE.kMaxRotation; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(MaxAngularRate.times(0.1)); // Add a 10% deadband

  private SwerveRequest.SwerveDriveBrake m_swerveDriveBrakeRequest =
      new SwerveRequest.SwerveDriveBrake();

  // Units per second, where 100% = 1 unit. Multiply by our robot's max speed to get m/s accel limit
  // TODO: Change these values
  // Needs seperate filters because each filter takes in different values
  // The robot base is also a square so acceleration limits will be different depending on direction
  private SlewRateLimiter driveXAccelLimiter =
      new SlewRateLimiter(SWERVE.kXAccelRateLimit, SWERVE.kXDeccelRateLimit, 0);
  private SlewRateLimiter driveYAccelLimiter =
      new SlewRateLimiter(SWERVE.kYAccelRateLimit, SWERVE.kYDeccelRateLimit, 0);

  private Robot2d m_robotSim = new Robot2d();
  private final Telemetry m_telemetry =
      new Telemetry(MaxSpeed.in(MetersPerSecond), SWERVE.kModuleTranslations);
  private FieldSim m_fieldSim;
  private FuelSim m_fuelSim;

  @Logged(name = "AutoChooser")
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  @Logged(name = "AutoSideChooser")
  private final SendableChooser<Boolean> m_autoSide = new SendableChooser<>();

  private Boolean m_flipToRight = false;

  @Logged(name = "ManualRPMShift", importance = Logged.Importance.INFO)
  private double m_manualRPMshift = 0.0;

  @Logged(name = "ManualHoodAngleShift", importance = Logged.Importance.INFO)
  private double m_manualHoodAngleShift = 0.0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    FIELD.initializeConstants();
    FIELD.updateConstants();
    ROBOT.initializeConstants();
    initializeSubSystems();
    configureBindings();
    initSmartDashboard();

    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
  }

  public Command setDrivetrainMode(NeutralModeValue neutralmode, MOTOR_TYPE motortype) {
    return m_swerveDrive.runOnce(() -> m_swerveDrive.setNeutralMode(motortype, neutralmode));
  }

  private void initializeSubSystems() {
    if (ROBOT.robotID.equals(ROBOT_ID.V1)) {
      m_swerveDrive = V1Constants.createDrivetrain();
    } else {
      m_swerveDrive = V2Constants.createDrivetrain();
    }
    m_swerveDrive.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_swerveDrive.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MaxSpeed.times(
                            -m_driverController
                                .getLeftY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            -m_driverController.getLeftX())) // Drive left with negative X (left)
                    .withRotationalRate(MaxAngularRate.times(-m_driverController.getRightX()))));
    m_flywheel = new Flywheel();
    m_controls = new Controls();
    m_vision = new Vision(m_controls);
    m_hood = new Hood();
    m_intake = new Intake();
    m_uptake = new Uptake();
    m_indexer = new Indexer();
    if (!ROBOT.robotID.equals(ROBOT_ID.V1) || RobotBase.isSimulation()) {
      m_intakePivot = new IntakePivot();
      m_led = new LEDs();
      m_led.setDefaultCommand(new UpdateLEDs(m_led, m_intake, m_flywheel));
      // m_led.setDefaultCommand(new TestLEDs(m_led));
      // m_climber = new Climber();
    }

    if (Robot.isSimulation()) {
      m_fuelSim = new FuelSim();
      m_fieldSim = new FieldSim();
      m_telemetry.registerFieldSim(m_fieldSim);
      m_vision.registerFieldSim(m_fieldSim);
      m_telemetry.registerFieldSim(m_fieldSim);
      FIELD.plotAllPositions(m_fieldSim);
      m_robotSim.registerSubsystems(
          m_intake, m_intakePivot, m_indexer, m_uptake, m_flywheel, m_hood);

      DriverStation.silenceJoystickConnectionWarning(true);
    }
    m_vision.registerSwerveDrive(m_swerveDrive);
    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
  }

  private ParallelCommandGroup resetManualShifts() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> m_manualRPMshift = 0.0),
        new InstantCommand(() -> m_manualHoodAngleShift = 0.0));
  }

  private void configureBindings() {
    if (m_intake != null)
      m_driverController.a().whileTrue(m_intake.commandIntakeState(INTAKE_STATE.REVERSING));

    POVUtils.povUpWithTilt(m_driverController)
        .whileTrue(m_swerveDrive.autoCrossBump(() -> m_vision.updateCrossBumpPath()));

    if (m_flywheel != null && m_hood != null) {
      m_driverController
          .x()
          .whileTrue(
              new ParallelCommandGroup(
                  m_flywheel.manualAgainstHubCommand(), m_hood.manualAgainstHubCommand()));
    }

    if (m_flywheel != null && m_hood != null && m_vision != null && m_swerveDrive != null) {
      m_driverController
          .leftBumper()
          .whileTrue(
              new Shoot(
                  m_flywheel,
                  m_hood,
                  m_vision,
                  m_driverController,
                  m_swerveDrive,
                  m_driverController::getLeftY,
                  m_driverController::getLeftX,
                  () -> m_manualHoodAngleShift,
                  () -> m_manualRPMshift));
    }

    if (m_intake != null) {
      m_driverController
          .leftTrigger()
          .whileTrue(new IntakeCommand(m_intake, m_intakePivot, m_uptake));
    }
    if (m_intake != null) {
      m_driverController
          .y()
          .or(m_operatorController.y())
          .whileTrue(new JostleIntake(m_intakePivot));
    }

    if (m_intakePivot != null) {
      Trigger manualOverrideActivate =
          new Trigger(() -> (Math.abs(m_operatorController.getLeftY()) > 0.1));
      manualOverrideActivate.whileTrue(
          m_intakePivot.manualOpenLoopOverride(m_operatorController::getLeftY));
    }

    m_driverController.rightTrigger().whileTrue(new Fire(m_intake, m_indexer, m_uptake));

    POVUtils.povDownWithTilt(m_driverController)
        .whileTrue(m_swerveDrive.applyRequest(() -> m_swerveDriveBrakeRequest));

    // Decrease rpm manual shift by 0.2
    m_operatorController
        .leftBumper()
        .onTrue(new InstantCommand(() -> m_manualRPMshift -= (FLYWHEEL.rpmShiftIncrement.in(RPM))));
    // Increase rpm manual shift by 0.2
    m_operatorController
        .rightBumper()
        .onTrue(new InstantCommand(() -> m_manualRPMshift += (FLYWHEEL.rpmShiftIncrement.in(RPM))));

    // Decrease hood angle manual shift by 0.2
    // POVUtils.povDownWithTilt(m_operatorController).onTrue(
    //     new InstantCommand(() -> m_manualHoodAngleShift -=
    // (FLYWHEEL.HOOD.angleShiftIncrement.in(Degrees)))
    // );
    // // Increase hood angle manual shift by 0.2
    // POVUtils.povUpWithTilt(m_operatorController).onTrue(
    //     new InstantCommand(() -> m_manualHoodAngleShift +=
    // (FLYWHEEL.HOOD.angleShiftIncrement.in(Degrees)))
    // );

    m_operatorController.b().onTrue(resetManualShifts());

    POVUtils.povDownWithTilt(m_operatorController)
        .whileTrue(m_intakePivot.command(PIVOT_SETPOINT.DEFUEL));

    // if (m_swerveDrive != null) {
    //   m_driverController.a().whileTrue(setDrivetrainMode(NeutralModeValue.Coast,
    // MOTOR_TYPE.STEER));
    //   m_driverController.b().whileTrue(setDrivetrainMode(NeutralModeValue.Brake,
    // MOTOR_TYPE.STEER));
    // }
  }

  private void initAutoChooser() {
    SmartDashboard.putData("Auto Mode", m_autoChooser);
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));

    var autoDeps =
        new AutoDependencies(
            m_swerveDrive,
            m_intake,
            m_vision,
            m_flywheel,
            m_hood,
            m_intakePivot,
            m_indexer,
            m_uptake);

    IntakeFromNeutral.registerNamedCommands(autoDeps);

    m_autoChooser.addOption("Center Preload", new CenterPreload(autoDeps));
    m_autoChooser.addOption("Simbotics Auto", new SimboticsAuto(autoDeps));
    m_autoChooser.addOption(
        "Two Cycle Conservative", new TwoCycle(autoDeps, () -> m_flipToRight, false));
    m_autoChooser.addOption("Two Cycle", new TwoCycleRush(autoDeps, () -> m_flipToRight, false));
    m_autoChooser.addOption(
        "Two Cycle - Alliance Partner Friendly", new TwoCycle(autoDeps, () -> m_flipToRight, true));
    m_autoChooser.addOption(
        "Two Cycle - Inside Out Alliance Partner Friendly",
        new TwoCycleInsideOutRush(autoDeps, () -> m_flipToRight, true));
    m_autoChooser.addOption(
        "Two Cycle - Inside Out", new TwoCycleInsideOutRush(autoDeps, () -> m_flipToRight, false));
    m_autoChooser.addOption(
        "Two Cycle Delay", new NextLevelAuto(autoDeps, () -> m_flipToRight, true));
    m_autoChooser.addOption(
        "Single Scoop with Sprinkles (Depot)",
        new SingleScoopWithSprinkles(autoDeps, () -> m_flipToRight, true));
    m_autoChooser.addOption("Just Depot", new JustDepot(autoDeps, () -> m_flipToRight));
    // m_autoChooser.addOption("Two Cycle (Rush) - Inside Out Conservative", new
    // TwoCycleInsideOutConservativeRush(autoDeps, () -> m_flipToRight, false));
    m_autoChooser.addOption(
        "Two Cycle - Rush", new TwoCycleRush(autoDeps, () -> m_flipToRight, false));
    m_autoChooser.addOption(
        "Two Cycle (Rush) - Alliance Partner Friendly",
        new TwoCycle(autoDeps, () -> m_flipToRight, true));
    m_autoChooser.addOption(
        "Test - Intake from Neutral",
        new IntakeFromNeutral(autoDeps, () -> m_flipToRight, TWO_CYCLE_PATH.FIRST_PASS));
  }

  private void initSideChooser() {
    SmartDashboard.putData("Auto Side", m_autoSide);
    m_autoSide.setDefaultOption("No Flip", false);

    m_autoSide.addOption("Depot", false);
    m_autoSide.addOption("Outpost", true);
    m_autoSide.onChange((Boolean selected) -> m_flipToRight = selected);
  }

  public void simulationPeriodic() {
    m_fuelSim.updateSim();
    // SmartDashboard.putNumber("Current Red Score:",
    // m_fuelSim.Hub.RED_HUB.getScore());
    // SmartDashboard.putNumber("Current Blue Score:",
    // m_fuelSim.Hub.BLUE_HUB.getScore());
  }

  private void initSmartDashboard() {
    initAutoChooser();
    initSideChooser();
    SmartDashboard.putData("ResetGyro", new ResetGyro(m_swerveDrive));
    if (RobotBase.isSimulation()) {
      SmartDashboard.putData(
          "Start Fuel Sim", new InstantCommand((this::initializeFuelSim)).ignoringDisable(true));
      SmartDashboard.putData(
          "Reset Fuel Sim", new InstantCommand((this::resetFuelSim)).ignoringDisable(true));
    }

    SmartDashboard.putData("Start Signal Logger", Commands.runOnce(SignalLogger::start));
    SmartDashboard.putData("Stop Signal Logger", Commands.runOnce(SignalLogger::stop));

    SmartDashboard.putData(
        "SysID Quasistatic Forward",
        m_swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData(
        "SysID Quasistatic Reverse",
        m_swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData(
        "SysID Dynamic Forward", m_swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData(
        "SysID Dynamic Reverse", m_swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public void testInit() {
    if (m_flywheel != null) m_flywheel.testInit();
    if (m_vision != null) m_vision.testInit();
    // if (m_uptake != null) m_uptake.testInit();
    // if (m_indexer != null) m_indexer.testInit();
    if (m_intakePivot != null) m_intakePivot.testInit();
    // if (m_intake != null) m_intake.testInit();
    if (m_hood != null) m_hood.testInit();
  }

  public void testPeriodic() {
    if (m_flywheel != null) m_flywheel.testPeriodic();
    // if (m_uptake != null) m_uptake.testPeriodic();
    // if (m_indexer != null) m_indexer.testPeriodic();
    if (m_intakePivot != null) m_intakePivot.testPeriodic();
    // if (m_intake != null) m_intake.testPeriodic();
    if (m_hood != null) m_hood.testPeriodic();
  }

  public void disabledPeriodic() {
    if (m_vision != null) m_vision.disabledPeriodic();
  }

  public void autonomousPeriodic() {
    System.out.println(
        "Flywheel RPM ready: "
            + m_flywheel.isAtRPMsetpoint()
            + ", Hood at setpoint: "
            + m_hood.atSetpoint()
            + ", Vision on target: "
            + m_vision.isOnTarget());
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

  public void robotPeriodic() {
    FIELD.updateCurrentSector(m_swerveDrive.getState().Pose);
  }

  public void initializeFuelSim() {
    if (m_fuelSim.isRunning()) {
      resetFuelSim();
    } else {
      m_fuelSim.spawnStartingFuel(); // spawns fuel in the depots and neutral zone
      m_fuelSim.registerRobot(
          SWERVE.kTrackWidth.in(Meters), // from left to right
          SWERVE.kWheelBase.in(Meters), // from front to back
          SWERVE.kBumperHeight.in(Meters), // from floor to top of bumpers
          () -> m_swerveDrive.getState().Pose, // Supplier<Pose2d> of robot pose
          () ->
              m_swerveDrive.getState()
                  .Speeds); // Supplier<ChassisSpeeds> of field-centric chassis speeds
      m_fuelSim
          .start(); // enables the simulation to run (updateSim must still be called periodically)
      m_fuelSim.registerIntake(
          Inches.of(13.688).in(Meters),
          Inches.of(15.094).in(Meters),
          Inches.of(-13.938).in(Meters),
          Inches.of(23.388).in(Meters),
          () -> m_intake.getStoredFuel() <= SIM.MAX_FUEL && m_intake.isIntaking(),
          () -> {
            m_intake.setStoredFuel(m_intake.getStoredFuel() + 1);
            System.out.println("Intaked fuel! New fuel count: " + m_intake.getStoredFuel());
          });
    }
  }

  public void updateFuelLaunchSim() {
    // If uptake and flywheel are running, launch fuel from the sim
    if (m_uptake != null && m_flywheel != null && m_hood != null) {
      // if (m_uptake.getMotorSpeedRPM() > (UPTAKE_SPEED.SHOOTING.get().in(RPM) * 0.90) // TODO:
      // Reimplement
      //     && m_intake.getStoredFuel() > 0) {
      // ReCalc and Desmos estimated this equation to convert RPM to linear velocity
      // of the fuel
      // vel in ft/s = 0.0111882 * RPM - 0.
      try {
        m_intake.setStoredFuel(m_intake.getStoredFuel() - 1);
        m_fuelSim.launchFuel(
            FeetPerSecond.of(m_flywheel.getMotorSpeedRPM() * 0.0111882 - 0.000174927),
            m_hood.getHoodAngle(),
            Degrees.of(0),
            FLYWHEEL.fuelLaunchHeight);
        System.out.println(
            "Launching fuel at velocity: "
                + (m_flywheel.getMotorSpeedRPM() * 0.0111882 - 0.000174927)
                + " ft/s and angle: "
                + m_hood.getHoodAngleDegrees()
                + " degrees");
        System.out.println("Launched fuel! Remaining fuel: " + m_intake.getStoredFuel());
      } catch (IllegalStateException e) {
        return;
      }
      // }
    }
  }

  public void resetFuelSim() {
    m_fuelSim.clearFuel();
    m_fuelSim.spawnStartingFuel();
    if (m_intake != null) m_intake.setStoredFuel(8); // preload
  }
}
