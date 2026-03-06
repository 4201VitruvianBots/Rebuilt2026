package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FIELD;
import frc.robot.constants.SWERVE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class AutoAlignDrive extends Command {
  private final CommandSwerveDrivetrain m_swerveDrivetrain;
  private final Vision m_vision;

  private final DoubleSupplier m_throttleInput;
  private final DoubleSupplier m_strafeInput;
  private boolean ifZeroFacesBump;

  /** Creates a new AutoAlign. */
  public AutoAlignDrive(
      CommandSwerveDrivetrain commandSwerveDrivetrain,
      Vision vision,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput) {
    m_vision = vision;
    m_swerveDrivetrain = commandSwerveDrivetrain;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    addRequirements(m_swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ifZeroFacesBump =
        (m_vision.isInBlueSector()
            || FIELD.getCurrentSector() == FIELD.SECTOR.NEUTRAL_FAR_LEFT
            || FIELD.getCurrentSector() == FIELD.SECTOR.NEUTRAL_FAR_RIGHT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrivetrain.setChassisSpeedsWithHeading(
        SWERVE.kMaxSpeedBump.times(m_throttleInput.getAsDouble()),
        SWERVE.kMaxSpeedBump.times(m_strafeInput.getAsDouble()),
        ifZeroFacesBump ? Rotation2d.kZero : Rotation2d.k180deg);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
