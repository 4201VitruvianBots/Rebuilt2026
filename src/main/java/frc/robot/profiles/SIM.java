package frc.robot.profiles;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import frc.robot.constants.CAN;

public class SIM extends V1 {

  public static void init() {
    V1.init();

    robotId = ROBOT_ID.SIM;
    robotName = robotId.getName();
    logMode = Logged.Importance.DEBUG;

    CAN.roboRIO = new CANBus("rio", "./logs/example.hoot");
    CAN.canivore = CAN.roboRIO;
  }
}
