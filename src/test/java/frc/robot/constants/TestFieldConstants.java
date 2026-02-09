package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Controls;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;
import utils.TestUtils;

public class TestFieldConstants {

  @Test
  public void testFieldConstants() {
    FIELD.initializeConstants();

    assertNotEquals(FIELD.TOWER.RED.CENTER, FIELD.TOWER.BLUE.CENTER);
    assertNotEquals(FIELD.ZONE.BLUE.ZONE, FIELD.ZONE.RED.ZONE);
  }

  @Test
  public void testSectorMap() {
    Controls controls = new Controls();
    FIELD.initializeConstants();
    FIELD.updateConstants();

    Map<Pose2d, FIELD.SECTOR> redSectors = new HashMap<>();
    redSectors.put(new Pose2d(1, 1, Rotation2d.kZero), FIELD.SECTOR.BLUE_LEFT);
    redSectors.put(new Pose2d(1, 8, Rotation2d.kZero), FIELD.SECTOR.BLUE_RIGHT);
    redSectors.put(new Pose2d(5, 1, Rotation2d.kZero), FIELD.SECTOR.NEUTRAL_FAR_LEFT);
    redSectors.put(new Pose2d(5, 8, Rotation2d.kZero), FIELD.SECTOR.NEUTRAL_FAR_RIGHT);
    redSectors.put(new Pose2d(9, 1, Rotation2d.kZero), FIELD.SECTOR.NEUTRAL_NEAR_LEFT);
    redSectors.put(new Pose2d(9, 8, Rotation2d.kZero), FIELD.SECTOR.NEUTRAL_NEAR_RIGHT);
    redSectors.put(new Pose2d(13, 1, Rotation2d.kZero), FIELD.SECTOR.RED_LEFT);
    redSectors.put(new Pose2d(13, 8, Rotation2d.kZero), FIELD.SECTOR.RED_RIGHT);

    redSectors.forEach(
        (k, v) -> {
          FIELD.updateCurrentSector(k);
          assertEquals(v, FIELD.getCurrentSector());
        });

    // Test Blue Alliance
    TestUtils.setPrivateField(controls, "m_allianceColor", DriverStation.Alliance.Blue);
    FIELD.updateConstants();
    Map<Pose2d, FIELD.SECTOR> blueSectors = new HashMap<>();
    blueSectors.put(new Pose2d(1,1, Rotation2d.kZero), FIELD.SECTOR.BLUE_RIGHT);
    blueSectors.put(new Pose2d(1,8, Rotation2d.kZero), FIELD.SECTOR.BLUE_LEFT);
    blueSectors.put(new Pose2d(5,1, Rotation2d.kZero), FIELD.SECTOR.NEUTRAL_NEAR_RIGHT);
    blueSectors.put(new Pose2d(5,8, Rotation2d.kZero), FIELD.SECTOR.NEUTRAL_NEAR_LEFT);
    blueSectors.put(new Pose2d(9,1, Rotation2d.kZero), FIELD.SECTOR.NEUTRAL_FAR_RIGHT);
    blueSectors.put(new Pose2d(9,8, Rotation2d.kZero), FIELD.SECTOR.NEUTRAL_FAR_LEFT);
    blueSectors.put(new Pose2d(13,1, Rotation2d.kZero), FIELD.SECTOR.RED_RIGHT);
    blueSectors.put(new Pose2d(13,8, Rotation2d.kZero), FIELD.SECTOR.RED_LEFT);

    blueSectors.forEach(
        (k, v) -> {
          FIELD.updateCurrentSector(k);
          assertEquals(v, FIELD.getCurrentSector());
        });
  }
}
