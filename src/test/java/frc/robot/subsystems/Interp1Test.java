package frc.robot.subsystems;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Test;

import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.HoodSubsystem;

public class Interp1Test {
    @Test
    void interp1Test() {
        assertEquals(HoodSubsystem.interp1(0, 1, 0, 10, 0), 0); // boundary case
        // assertEquals(HoodSubsystem.interp1(0, 1, 0, 10, -1), 0); // clamps outside bounds
        assertEquals(HoodSubsystem.interp1(0, 1, 0, 10, 1), 10); // boundary case
        // assertEquals(HoodSubsystem.interp1(0, 1, 0, 10, 2), 10); // clamps outside bounds
        assertEquals(HoodSubsystem.interp1(0, 1, 0, 10, 0.5), 5); // midpoint check
        assertEquals(HoodSubsystem.interp1(10, 20, 0, 10, 15), 5); // midpoint check
        assertEquals(HoodSubsystem.interp1(10, 20, 20, 30, 15), 25); // fully general midpoint check

        assertEquals(HoodSubsystem.interp1(20,10, 20, 30, 18), 22); // check reverse line

    }
}
