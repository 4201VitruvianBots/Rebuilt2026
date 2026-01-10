package frc.team4201.lib.utils;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

/** Utility class to interact with CTRE libraries. */
public final class CtreUtils {
  /**
   * Initialize Phoenix Server by creating a dummy device. We do this so that the CANCoders don't
   * get configured before Phoenix Server is up, which causes issues with encoder offsets not being
   * set/applied properly.
   */
  public static void initPhoenixServer() {
    var alert =
        new Alert("Starting Phoenix Server at: " + Timer.getFPGATimestamp(), AlertType.kInfo);
    alert.set(true);
    if (RobotBase.isReal()) {
      TalonFX dummy = new TalonFX(0, new CANBus("rio"));
      Timer.delay(5);
      dummy.close();
      dummy = null;
    }
    alert.setText("Phoenix Server finished Init at: " + Timer.getFPGATimestamp());
  }

  /**
   * Apply a {@link TalonFXConfiguration} to a {@link TalonFX}. Will retry until it gets
   * StatusCode.OK or until it gives up after 5 tries. If it gives up, an {@link Alert} will be sent
   * to warn the user that it has failed.
   *
   * @param motor TalonFX to configure
   * @param config TalonFXConfiguration to apply
   * @return boolean true if successful
   */
  public static boolean configureTalonFx(TalonFX motor, TalonFXConfiguration config) {
    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < (RobotBase.isReal() ? 5 : 1); i++) {
      motorStatus = motor.getConfigurator().apply(config);
      if (motorStatus.isOK()) break;
      if (RobotBase.isReal()) Timer.delay(0.02);
    }
    if (!motorStatus.isOK()) {
      var alert =
          new Alert(
              String.format(
                  "Could not apply configs to TalonFx ID: %d. Error code: %s",
                  motor.getDeviceID(), motorStatus),
              AlertType.kError);
      alert.set(true);
    } else System.out.printf("TalonFX ID: %d - Successfully configured!\n", motor.getDeviceID());

    return motorStatus.isOK();
  }

  /**
   * Apply a {@link CANcoderConfiguration} to a {@link CANcoder}. Will retry until it gets
   * StatusCode.OK or until it gives up after 5 tries. If it gives up, an {@link Alert} will be sent
   * to warn the user that it has failed.
   *
   * @param cancoder CANcoder to configure
   * @param config CANcoderConfiguration to apply
   * @return boolean true if successful
   */
  public static boolean configureCANCoder(CANcoder cancoder, CANcoderConfiguration config) {
    StatusCode canCoderStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < (RobotBase.isReal() ? 5 : 1); i++) {
      canCoderStatus = cancoder.getConfigurator().apply(config);
      if (canCoderStatus.isOK()) break;
      if (RobotBase.isReal()) Timer.delay(0.02);
    }
    if (!canCoderStatus.isOK()) {
      var alert =
          new Alert(
              String.format(
                  "Could not apply configs to CANCoder ID: %d. Error code: %s",
                  cancoder.getDeviceID(), canCoderStatus),
              AlertType.kError);
      alert.set(true);
    } else
      System.out.printf("CANCoder ID: %d - Successfully configured!\n", cancoder.getDeviceID());
    return canCoderStatus.isOK();
  }
}
