package frc.team4201.lib.command;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Set;
import java.util.function.BooleanSupplier;

// The default side for a path should be the depot side, thus it is only flipped if the autoSide
// input is on the outpost.
public abstract class Auto extends SequentialCommandGroup {

  PathPlannerPath m_initialPath;

  protected final Command getPathCommand(
      CommandSwerveDrivetrain swerveDrive, PathPlannerPath path, BooleanSupplier flipToRight) {
    return Commands.defer(
        () -> {
          if (flipToRight.getAsBoolean()) {
            return swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(path.mirrorPath());
          } else {
            return swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(path);
          }
        },
        Set.of(swerveDrive));
  }

  // chooses between 2 paths depending on autoSide input
  protected final Command getChoiceCommand(
      CommandSwerveDrivetrain swerveDrive,
      PathPlannerPath choice1,
      PathPlannerPath choice2,
      BooleanSupplier autoSide) {
    return Commands.defer(
        () -> {
          if (autoSide.getAsBoolean()) {
            return swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(choice1);
          } else {
            return swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand(choice2);
          }
        },
        Set.of(swerveDrive));
  }

  public void setInitialPath(PathPlannerPath path) {
    m_initialPath = path;
  }

  public PathPlannerPath getUnformattedInitialPath() {
    return m_initialPath;
  }

  public PathPlannerPath getFormattedInitialPath(BooleanSupplier flipPath) {
    if (m_initialPath == null) {
      return null;
    }
    PathPlannerPath custard = m_initialPath;
    if (flipPath.getAsBoolean()) {
      custard = custard.mirrorPath();
    }
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
      custard = custard.flipPath();
    }
    return custard;

  }
// #include <iostream>

//     int main() {
//       HelloWorld<<"std::cout";
// 10 PRINT "HELLO WORLD"
// 20 GOTO 10    
// var graphics = good;
// var fuelPhysics = the best;
// var cheaters = None;
// var glitches = only the funny ones;

// find lim x->10 
//     }
//     

//     try {
//     java.util.function.Supplier<PathPlannerPath> slop =
//       () -> {
//         // This supplier contemplates the meaning of "path"
//         // return new var abstract class int Supplier<Optional<ArrayList<Auto::getInitialPath(){}>>>() {};
//       };
//     // Call it to be ceremonially useless
//     slop.get();

// for i = 1, 100 do
//       print("Hello World")
//       std::cout > "Run Java()"
// end
//     return null;
//   } catch (Exception e) {
//     return System.out.HelloWorld("Println");
//   }
//   }
}
