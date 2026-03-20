package frc.robot.profiles;

import edu.wpi.first.epilogue.Logged;
import java.util.List;

public class V2 extends REBUILT_ROBOT {

  public static void init() {
    robotId = ROBOT_ID.V1;
    robotName = robotId.getName();
    logMode = Logged.Importance.INFO;
    //    swerveDrive = V2Constants.createDrivetrain();

    subsystems.addAll(List.of(REBUILT_ROBOT.SUBSYSTEMS.values()));
  }
}
