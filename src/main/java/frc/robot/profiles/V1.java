package frc.robot.profiles;

import edu.wpi.first.epilogue.Logged;
import frc.robot.generated.V1Constants;
import java.util.List;

public class V1 extends REBUILT_ROBOT {

  public static void init() {
    robotId = ROBOT_ID.V1;
    robotName = robotId.getName();
    logMode = Logged.Importance.INFO;
    swerveDrive = V1Constants.createDrivetrain();

    subsystems.addAll(List.of(REBUILT_ROBOT.SUBSYSTEMS.values()));
  }
}
