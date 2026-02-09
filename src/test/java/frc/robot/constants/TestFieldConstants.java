package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class TestFieldConstants {

  @Test
  public void testFieldConstants() {
    FIELD.initializeConstants();

    assertNotEquals(FIELD.TOWER.RED.CENTER, FIELD.TOWER.BLUE.CENTER);
    assertNotEquals(FIELD.ZONE.BLUE.ZONE, FIELD.ZONE.RED.ZONE);
  }
}
