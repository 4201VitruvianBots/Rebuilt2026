package frc.team4201.lib.hardwareMonitor.configs;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.units.measure.*;
import frc.robot.generated.CommandSwerveDrivetrain;

public abstract class CtreSwerveConstants {
  // Both sets of gains need to be tuned to your individual robot.

  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  protected static final Slot0Configs steerGains = new Slot0Configs();

  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  protected static final Slot0Configs driveGains = new Slot0Configs();

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  protected static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  protected static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The type of motor used for the drive motor
  protected static final DriveMotorArrangement kDriveMotorType =
      DriveMotorArrangement.TalonFX_Integrated;
  // The type of motor used for the drive motor
  protected static final SteerMotorArrangement kSteerMotorType =
      SteerMotorArrangement.TalonFX_Integrated;

  // The remote sensor feedback type to use for the steer motors;
  // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
  protected static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  protected static final Current kSlipCurrent = Amps.of(120);

  // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
  protected static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  protected static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration();

  protected static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  protected static final Pigeon2Configuration pigeonConfigs = null;

  // CAN bus that the devices are located on;
  // All swerve devices must share the same CAN bus
  public static final CANBus kCANBus = new CANBus();

  // Theoretical free speed (m/s) at 12 V applied output;
  // This needs to be tuned to your individual robot
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.zero();

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  protected static final double kCoupleRatio = 4.5;

  protected static final double kDriveGearRatio = 7.03125;
  protected static final double kSteerGearRatio = 26.09090909090909;
  protected static final Distance kWheelRadius = Inches.of(2);

  protected static final boolean kInvertLeftSide = false;
  protected static final boolean kInvertRightSide = true;

  protected static final int kPigeonId = 9;

  // These are only used for simulation
  protected static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
  protected static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
  // Simulated voltage necessary to overcome friction
  protected static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  protected static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants();

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator = new SwerveModuleConstantsFactory<>();

  // Front Left
  protected static final int kFrontLeftDriveMotorId = 20;
  protected static final int kFrontLeftSteerMotorId = 21;
  protected static final int kFrontLeftEncoderId = 10;
  protected static final Angle kFrontLeftEncoderOffset = Rotations.zero();
  protected static final boolean kFrontLeftSteerMotorInverted = false;
  protected static final boolean kFrontLeftEncoderInverted = false;

  protected static final Distance kFrontLeftXPos = Inches.zero();
  protected static final Distance kFrontLeftYPos = Inches.zero();

  // Front Right
  protected static final int kFrontRightDriveMotorId = 0;
  protected static final int kFrontRightSteerMotorId = 0;
  protected static final int kFrontRightEncoderId = 0;
  protected static final Angle kFrontRightEncoderOffset = Rotations.of(0.239990234375);
  protected static final boolean kFrontRightSteerMotorInverted = false;
  protected static final boolean kFrontRightEncoderInverted = false;

  private static final Distance kFrontRightXPos = Inches.of(8.875);
  private static final Distance kFrontRightYPos = Inches.of(-12.625);

  // Back Left
  private static final int kBackLeftDriveMotorId = 24;
  private static final int kBackLeftSteerMotorId = 25;
  private static final int kBackLeftEncoderId = 12;
  private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.01123046875);
  private static final boolean kBackLeftSteerMotorInverted = false;
  private static final boolean kBackLeftEncoderInverted = false;

  private static final Distance kBackLeftXPos = Inches.of(-8.875);
  private static final Distance kBackLeftYPos = Inches.of(12.625);

  // Back Right
  private static final int kBackRightDriveMotorId = 26;
  private static final int kBackRightSteerMotorId = 27;
  private static final int kBackRightEncoderId = 13;
  private static final Angle kBackRightEncoderOffset = Rotations.of(0.4248046875);
  private static final boolean kBackRightSteerMotorInverted = false;
  private static final boolean kBackRightEncoderInverted = false;

  private static final Distance kBackRightXPos = Inches.of(-8.875);
  private static final Distance kBackRightYPos = Inches.of(-12.625);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              kFrontLeftXPos,
              kFrontLeftYPos,
              kInvertLeftSide,
              kFrontLeftSteerMotorInverted,
              kFrontLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              kFrontRightXPos,
              kFrontRightYPos,
              kInvertRightSide,
              kFrontRightSteerMotorInverted,
              kFrontRightEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              kBackLeftXPos,
              kBackLeftYPos,
              kInvertLeftSide,
              kBackLeftSteerMotorInverted,
              kBackLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              kBackRightXPos,
              kBackRightYPos,
              kInvertRightSide,
              kBackRightSteerMotorInverted,
              kBackRightEncoderInverted);

  /**
   * Creates a CommandSwerveDrivetrain instance. This should only be called once in your robot
   * program,.
   */
  public static CommandSwerveDrivetrain createDrivetrain() {
    return new CommandSwerveDrivetrain(
        DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
  }

  /** Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types. */
  public abstract static class TunerSwerveDrivetrain
      extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    }
  }
}
