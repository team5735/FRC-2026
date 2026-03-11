package frc.robot.subsystems;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class Interp1Test {
    @Test
    void interp1Test() {
        assertEquals(HoodSubsystem.interp1(0, 1, 0, 10, 0), 0); // boundary case
        assertEquals(HoodSubsystem.interp1(0, 1, 0, 10, -1), -10); // out of bounds case
        assertEquals(HoodSubsystem.interp1(0, 1, 0, 10, 1), 10); // boundary case
        assertEquals(HoodSubsystem.interp1(0, 1, 0, 10, 2), 20); // out of bounds case
        assertEquals(HoodSubsystem.interp1(0, 1, 0, 10, 0.5), 5); // midpoint check
        assertEquals(HoodSubsystem.interp1(10, 20, 0, 10, 15), 5); // midpoint check
        assertEquals(HoodSubsystem.interp1(10, 20, 20, 30, 15), 25); // fully general midpoint check
        assertEquals(HoodSubsystem.interp1(20,10, 20, 30, 18), 22); // check reverse line
    }
}
