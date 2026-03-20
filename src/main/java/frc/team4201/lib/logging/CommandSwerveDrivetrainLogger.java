package frc.team4201.lib.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import frc.robot.generated.CommandSwerveDrivetrain;

@CustomLoggerFor(CommandSwerveDrivetrain.class)
public class CommandSwerveDrivetrainLogger extends ClassSpecificLogger<CommandSwerveDrivetrain> {

  public CommandSwerveDrivetrainLogger() {
    super(CommandSwerveDrivetrain.class);
  }

  @Override
  public void update(EpilogueBackend backend, CommandSwerveDrivetrain swerveDrive) {
    //    var currentRequest = swerveDrive.getCurrentRequest();
    //    backend.log("Current Swerve Request", currentRequest.getClass().getName());
  }
}
