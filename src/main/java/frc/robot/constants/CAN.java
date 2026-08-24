package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class CAN { //ids were gotten from controls
  public static final CANBus drivetrain = CANBus.systemcore(0);
  public static final CANBus intake = CANBus.systemcore(1);
  public static final CANBus indexer = CANBus.systemcore(2);
  public static final CANBus hood = CANBus.systemcore(3);
  public static final CANBus shooter = CANBus.systemcore(4);

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

  public static final int kShooterHoodMotor = 34;

  public static final int kShooterRollerMotor1 = 40;
  public static final int kShooterRollerMotor2 = 41;
  public static final int kShooterRollerMotor3 = 42;
  public static final int kShooterRollerMotor4 = 43; //TODO: change to a value that is real

  public static final int kIndexerMotor1 = 51;
  public static final int kIndexerMotor2 = 50;
  public static final int kIndexerMotor3 = 52; //same with these
  public static final int kIndexerMotor4 = 58;

  public static final int kIntakeRollerMotor1 = 53;
  public static final int kIntakeRollerMotor2 = 54;

  public static final int kIntakePivotMotor = 55;

  public static final int kUptakeMotor = 57;
}
