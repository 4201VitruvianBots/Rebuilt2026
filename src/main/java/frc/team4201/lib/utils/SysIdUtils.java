package frc.team4201.lib.utils;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// \import frc.team4201.lib.utils.ModuleMap.MODULE_POSITION;

public class SysIdUtils {
  private static SysIdRoutine swerveDriveRoutine;
  private static SysIdRoutine swerveTurnRoutine;

  // public static void createSwerveDriveRoutines(CommandSwerveDrivetrain swerveDrive) {
  //   swerveDriveRoutine =
  //       new SysIdRoutine(
  //           new SysIdRoutine.Config(
  //               null, null, null, (state) -> SignalLogger.writeString("state",
  // state.toString())),
  //           new Mechanism(
  //               (Voltage volts) -> {
  //                 for (var position : MODULE_POSITION.values()) {
  //                   var voltageControl = new VoltageOut(0);
  //                   swerveDrive
  //                       .getModule(position.ordinal())
  //                       .getDriveMotor()
  //                       .setControl(voltageControl.withOutput(volts.in(Volts)));
  //                 }
  //               },
  //               null,
  //               swerveDrive));
  // }

  // public static void createSwerveTurnRoutines(CommandSwerveDrivetrain swerveDrive) {
  //   var module = swerveDrive.getModule(MODULE_POSITION.FRONT_LEFT.ordinal());
  //   swerveTurnRoutine =
  //       new SysIdRoutine(
  //           new SysIdRoutine.Config(
  //               null, null, null, (state) -> SignalLogger.writeString("state",
  // state.toString())),
  //           new Mechanism(
  //               (Voltage volts) -> {
  //                 var voltageControl = new TorqueCurrentFOC(0);
  //                 module.getSteerMotor().setControl(voltageControl.withOutput(volts.in(Volts)));
  //               },
  //               null,
  //               swerveDrive));
  // }

  public static SysIdRoutine getSwerveDriveRoutine() {
    return swerveDriveRoutine;
  }

  public static SysIdRoutine getSwerveTurnRoutine() {
    return swerveTurnRoutine;
  }
}
