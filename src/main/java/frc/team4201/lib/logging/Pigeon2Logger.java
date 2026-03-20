package frc.team4201.lib.logging;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(Pigeon2.class)
public class Pigeon2Logger extends ClassSpecificLogger<Pigeon2> {

  public Pigeon2Logger() {
    super(Pigeon2.class);
  }

  @Override
  public void update(EpilogueBackend backend, Pigeon2 pigeon2) {
    backend.log("Yaw Angle", pigeon2.getYaw().getValue());
    backend.log("Yaw Velocity", pigeon2.getAngularVelocityYWorld().getValue());
    backend.log("Yaw Acceleration", pigeon2.getAccelerationZ().getValue());
  }
}
