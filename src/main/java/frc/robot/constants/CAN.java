package frc.robot.constants;

import static java.util.Map.entry;

import com.ctre.phoenix6.CANBus;
import java.util.EnumSet;
import java.util.Map;

public class CAN {
  public static CANBus roboRIO = CANBus.roboRIO();
  public static CANBus canivore = new CANBus("canivore", "./logs/example.hoot");

  public enum ID {
    PDH(0),

    PIGEON(9),

    FRONT_LEFT_CANCODER(10),
    FRONT_RIGHT_CANCODER(11),
    BACK_LEFT_CANCODER(12),
    BACK_RIGHT_CANCODER(13),

    FRONT_LEFT_DRIVE_MOTOR(20),
    FRONT_LEFT_TURN_MOTOR(21),
    FRONT_RIGHT_DRIVE_MOTOR(22),
    FRONT_RIGHT_TURN_MOTOR(23),
    BACK_LEFT_DRIVE_MOTOR(24),
    BACK_LEFT_TURN_MOTOR(25),
    BACK_RIGHT_DRIVE_MOTOR(26),
    BACK_RIGHT_TURN_MOTOR(27),

    CLIMBER_MOTOR(30),

    HOOD_MOTOR(34),
    HOOD_CANCODER(35),

    SHOOTER_MOTOR_1(40),
    SHOOTER_MOTOR_2(41),
    SHOOTER_MOTOR_3(42),

    INDEXER_MOTOR_1(50),
    INDEXER_MOTOR_2(51),

    INTAKE_MOTOR_1(53),
    INTAKE_MOTOR_2(54),

    INTAKE_PIVOT_MOTOR(55),
    INTAKE_PIVOT_CANCODER(56),

    UPTAKE_MOTOR(57);

    final int ID;

    ID(int id) {
      if (id < 0 || id > 63) {
        throw new IllegalArgumentException("Invalid CAN ID defined in CAN.ID!");
      }

      this.ID = id;
    }
  }

  public static final int pigeon = 9;

  public static final int frontLeftCanCoder = 10;
  public static final int frontRightCanCoder = 11;
  public static final int backLeftCanCoder = 12;
  public static final int backRightCanCoder = 13;

  public static final int frontLeftDriveMotor = 20;
  public static final int frontLeftTurnMotor = 21;
  public static final int frontRightDriveMotor = 22;
  public static final int frontRightTurnMotor = 23;
  public static final int backLeftDriveMotor = 24;
  public static final int backLeftTurnMotor = 25;
  public static final int backRightDriveMotor = 26;
  public static final int backRightTurnMotor = 27;

  public static final int kClimberMotor = 30;

  public static final int kShooterHoodMotor = 34;
  public static final int kShooterHoodCANCoder = 35;

  public static final int kShooterRollerMotor1 = 40;
  public static final int kShooterRollerMotor2 = 41;
  public static final int kShooterRollerMotor3 = 42;
  public static final int kShooterRollerMotor4 = 43;

  public static final int kIndexerMotor1 = 50;
  public static final int kIndexerMotor2 = 51;
  public static final int kIndexerMotor3 = 52;

  public static final int kIntakeRollerMotor1 = 53;
  public static final int kIntakeRollerMotor2 = 54;

  public static final int kIntakePivotMotor = 55;
  public static final int kPivotEncoder = 56;

  public static final int kUptakeMotor = 57;

  // Note: The order of devices matters. Devices should be ordered starting from the CANBus start
  public static final Map<CANBus, EnumSet<ID>> mappedCanDevices =
      Map.ofEntries(
          entry(
              canivore,
              EnumSet.of(
                  ID.BACK_LEFT_TURN_MOTOR,
                  ID.BACK_LEFT_CANCODER,
                  ID.BACK_LEFT_DRIVE_MOTOR,
                  ID.FRONT_LEFT_TURN_MOTOR,
                  ID.FRONT_LEFT_CANCODER,
                  ID.FRONT_LEFT_DRIVE_MOTOR,
                  ID.PIGEON,
                  ID.FRONT_RIGHT_DRIVE_MOTOR,
                  ID.FRONT_RIGHT_CANCODER,
                  ID.FRONT_RIGHT_TURN_MOTOR,
                  ID.BACK_RIGHT_DRIVE_MOTOR,
                  ID.BACK_RIGHT_CANCODER,
                  ID.BACK_RIGHT_TURN_MOTOR
                  /*CLIMBER_CANRANGE*/ )),
          // TODO: Verify the order of devices on the canivore
          entry(
              roboRIO,
              EnumSet.of(
                  ID.INDEXER_MOTOR_1,
                  ID.INDEXER_MOTOR_2,
                  ID.INTAKE_PIVOT_MOTOR,
                  ID.INTAKE_PIVOT_CANCODER,
                  ID.INTAKE_MOTOR_1,
                  ID.INTAKE_MOTOR_2,
                  ID.HOOD_MOTOR,
                  ID.HOOD_CANCODER,
                  ID.SHOOTER_MOTOR_1,
                  ID.SHOOTER_MOTOR_2,
                  ID.SHOOTER_MOTOR_3,
                  ID.CLIMBER_MOTOR)));
}
