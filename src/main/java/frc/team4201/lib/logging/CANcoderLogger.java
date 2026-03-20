package frc.team4201.lib.logging;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(CANcoder.class)
public class CANcoderLogger extends ClassSpecificLogger<CANcoder> {

  public CANcoderLogger() {
    super(CANcoder.class);
  }

  @Override
  public void update(EpilogueBackend backend, CANcoder cancoder) {
    backend.log("Position", cancoder.getAbsolutePosition().getValue());
    backend.log("Velocity", cancoder.getVelocity().getValue());
  }
}
