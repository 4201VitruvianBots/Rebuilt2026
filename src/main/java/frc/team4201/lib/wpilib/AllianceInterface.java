package frc.team4201.lib.wpilib;

import frc.robot.subsystems.Controls;

public interface AllianceInterface {
  static boolean isBlue() {
    return Controls.isBlueAlliance();
  }
}
