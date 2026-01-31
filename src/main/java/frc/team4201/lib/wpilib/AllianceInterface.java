package frc.team4201.lib.wpilib;

import edu.wpi.first.wpilibj.DriverStation;

public interface AllianceInterface {
  void updateFields();

  static boolean isBlue() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue);
    } else {
      return false;
    }
  }
}
