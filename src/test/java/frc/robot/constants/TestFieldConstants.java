package frc.robot.constants;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class TestFieldConstants {

    @Test
    public void testFieldConstants() {
        FIELD.initializeConstants();

        assertNotEquals(FIELD.TOWER.RED.CENTER, FIELD.TOWER.BLUE.CENTER);
    }
}
