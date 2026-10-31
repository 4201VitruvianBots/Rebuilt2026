// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.FeetPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RPM;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.commands.autos.AutoShoot;
import frc.robot.lib.BLine.FollowPath;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.NotLogged;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.framework.RobotBase;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.command2.button.Trigger;
import org.wpilib.command2.sysid.SysIdRoutine;
import frc.robot.commands.Fire;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JostleIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.UpdateLEDs;
import frc.robot.commands.autos.AutoDependencies;
import frc.robot.commands.swerve.ResetGyro;
import frc.robot.constants.FIELD;
import frc.robot.constants.FLYWHEEL;
import frc.robot.constants.INTAKE.PIVOT.PIVOT_SETPOINT;
import frc.robot.constants.INTAKE.ROLLERS.INTAKE_STATE;
import frc.robot.constants.ROBOT;
import frc.robot.constants.ROBOT.ROBOT_ID;
import frc.robot.constants.ROBOT.SIM;
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
import frc.robot.subsystems.Vision;
import frc.team4201.lib.simulation.FieldSim;
import frc.team4201.lib.utils.HubTracker;
import frc.team4201.lib.utils.POVUtils;
import frc.team4201.lib.utils.CtreTelemetry;
import org.wpilib.driverstation.Gamepad.Axis;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

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

  @Logged(name = "LEDs", importance = Logged.Importance.INFO)
  private LEDs m_led;

  @Logged(name = "IntakePivot", importance = Logged.Importance.INFO)
  private IntakePivot m_intakePivot;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandNiDsXboxController m_driverController = new CommandNiDsXboxController(USB.driver_xBoxController);
  private final CommandNiDsXboxController m_operatorController =
      new CommandNiDsXboxController(USB.operator_xboxController);

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
  private final CtreTelemetry m_telemetry =
      new CtreTelemetry(MaxSpeed.in(MetersPerSecond), SWERVE.kModuleTranslations);
  private FieldSim m_fieldSim;

  private final Selectable<Command> m_autoChooser = new Selectable<>();

  private final Selectable<Boolean> m_autoSide = new Selectable<>();

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
    initTunables();

    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
  }

  public Command setDrivetrainMode(NeutralModeValue neutralmode, MOTOR_TYPE motortype) {
    return m_swerveDrive.runOnce(() -> m_swerveDrive.setNeutralMode(motortype, neutralmode));
  }

  private void initializeSubSystems() {
    m_swerveDrive = V2Constants.createDrivetrain();
    
    m_swerveDrive.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_swerveDrive.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MaxSpeed.times(
                            -m_driverController.getLeftY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            -m_driverController.getLeftX())) // Drive left with negative X (left)
                    .withRotationalRate(MaxAngularRate.times(-m_driverController.getRightX()))));
    m_flywheel = new Flywheel();
    m_controls = new Controls();
    m_vision = new Vision(m_controls);
    m_hood = new Hood();
    m_intake = new Intake();
    m_indexer = new Indexer();
    if (!ROBOT.robotID.equals(ROBOT_ID.V1) || RobotBase.isSimulation()) {
      m_intakePivot = new IntakePivot();
      m_led = new LEDs();
      m_led.setDefaultCommand(new UpdateLEDs(m_led, m_intake, m_flywheel));
      // m_led.setDefaultCommand(new TestLEDs(m_led));
    }

    if (Robot.isSimulation()) {
      m_fieldSim = new FieldSim();
      m_telemetry.registerFieldSim(m_fieldSim);
      m_vision.registerFieldSim(m_fieldSim);
      FIELD.plotAllPositions(m_fieldSim);
      m_robotSim.registerSubsystems(
          m_intake, m_intakePivot, m_indexer, m_flywheel, m_hood);

      DriverStationBackend.silenceJoystickConnectionAlert(true);
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

    POVUtils.povUpWithTilt(m_driverController.getHID())
        .whileTrue(m_swerveDrive.autoCrossBump(() -> m_vision.updateCrossBumpPath(false)));

    if (m_flywheel != null && m_hood != null) {
      m_driverController.rightBumper()
          .whileTrue(
              new ParallelCommandGroup(
                  m_flywheel.manualAgainstHubCommand(), m_hood.manualAgainstHubCommand()));
    }

    if (m_flywheel != null && m_hood != null) { // Doesn't use utils
      m_driverController
          .getHID().povLeft()
          .whileTrue(m_intakePivot.sendPivotDown());
    }

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
                () -> m_manualRPMshift
              )
          );

    if (m_intake != null) {
      m_driverController.leftTrigger().whileTrue(new IntakeCommand(m_intake, m_intakePivot));
    }

    if (m_intakePivot != null) {
      Trigger manualOverrideActivate =
          new Trigger(() -> (Math.abs(m_operatorController.getLeftY()) > 0.1));
      manualOverrideActivate.whileTrue(
          m_intakePivot.manualOpenLoopOverride(m_operatorController::getLeftY));
    }

    m_driverController.rightTrigger().whileTrue(new ParallelCommandGroup(new Fire(m_indexer, m_intakePivot), m_swerveDrive.applyRequest(() -> m_swerveDriveBrakeRequest)));

    POVUtils.povDownWithTilt(m_driverController.getHID())
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

    m_operatorController.a().onTrue(resetManualShifts());

    POVUtils.povDownWithTilt(m_operatorController.getHID())
        .whileTrue(m_intakePivot.command(PIVOT_SETPOINT.DEFUEL));

    // if (m_swerveDrive != null) {
    //   m_driverController.a().whileTrue(setDrivetrainMode(NeutralModeValue.Coast,
    // MOTOR_TYPE.STEER));
    //   m_driverController.b().whileTrue(setDrivetrainMode(NeutralModeValue.Brake,
    // MOTOR_TYPE.STEER));
    // }
  }

  private void initAutoChooser() {
    Tunables.publish("Auto Mode", m_autoChooser);
    m_autoChooser.addDefault("Do Nothing", new WaitCommand(0));

    var autoDeps =
        new AutoDependencies(
            m_swerveDrive,
            m_intake,
            m_vision,
            m_flywheel,
            m_hood,
            m_intakePivot,
            m_indexer);

    FollowPath.registerEventTrigger("shoot", new AutoShoot(autoDeps, 3));
    FollowPath.registerEventTrigger("intake", new IntakeCommand(m_intake, m_intakePivot));

    m_autoChooser.add("Bline Center Preload", m_swerveDrive.generateBLineCommand("center_preload", () -> m_flipToRight));
    m_autoChooser.add("Bline Depot Only", m_swerveDrive.generateBLineCommand("depot_only"));
    m_autoChooser.add("Trench Return", m_swerveDrive.generateBLineCommand("trench_return", () -> m_flipToRight));
    m_autoChooser.add("Bump Return", m_swerveDrive.generateBLineCommand("bump_return", () -> m_flipToRight));
    m_autoChooser.add("Bump Return Delay", new WaitCommand(6).andThen(m_swerveDrive.generateBLineCommand("bump_return_single_swipe_delay", () -> m_flipToRight)));
    m_autoChooser.add("pid", m_swerveDrive.generateBLineCommand("pidtuning").andThen(()->System.out.println("Finished Tuning")));
  }

  private void initSideChooser() {
    Tunables.publish("Auto Side", m_autoSide);
    m_autoSide.addDefault("No Flip", false);

    m_autoSide.add("Depot", false);
    m_autoSide.add("Outpost", true);
    m_autoSide.onChange((Boolean selected) -> m_flipToRight = selected);
  }

  public void simulationPeriodic() {

  }

  private void initTunables() {
    initAutoChooser();
    initSideChooser();
    Tunables.publish("ResetGyro", new ResetGyro(m_swerveDrive));

    Tunables.publish("Start Signal Logger", Commands.runOnce(SignalLogger::start));
    Tunables.publish("Stop Signal Logger", Commands.runOnce(SignalLogger::stop));

    Tunables.publish(
        "SysID Quasistatic Forward",
        m_swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.FORWARD));
    Tunables.publish(
        "SysID Quasistatic Reverse",
        m_swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.REVERSE));
    Tunables.publish(
        "SysID Dynamic Forward", m_swerveDrive.sysIdDynamic(SysIdRoutine.Direction.FORWARD));
    Tunables.publish(
        "SysID Dynamic Reverse", m_swerveDrive.sysIdDynamic(SysIdRoutine.Direction.REVERSE));
  }

  public void utilityInit() {
    if (m_flywheel != null) m_flywheel.utilityInit();
    if (m_vision != null) m_vision.utilityInit();
    // if (m_uptake != null) m_uptake.utilityInit();
    // if (m_indexer != null) m_indexer.utilityInit();
    if (m_intakePivot != null) m_intakePivot.utilityInit();
    // if (m_intake != null) m_intake.utilityInit();
    if (m_hood != null) m_hood.utilityInit();
  }

  public void utilityPeriodic() {
    if (m_flywheel != null) m_flywheel.utilityPeriodic();
    // if (m_uptake != null) m_uptake.utilityPeriodic();
    // if (m_indexer != null) m_indexer.utilityPeriodic();
    if (m_intakePivot != null) m_intakePivot.utilityPeriodic();
    // if (m_intake != null) m_intake.utilityPeriodic();
    if (m_hood != null) m_hood.utilityPeriodic();
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
    Telemetry.log("In Neutral Zone?", m_vision.isInNeutralSector());
  }

  public void configureTeleopCurrentLimits(){
    m_intake.setCurrentLimits(40, 100);
  }
}
