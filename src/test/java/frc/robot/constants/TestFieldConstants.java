package frc.robot.constants;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class TestFieldConstants {

    @Test
    public void testFieldConstants() {
        assertFalse(FIELD.TOWER.RED.CENTER.equals(FIELD.TOWER.BLUE.CENTER));
    }
}
