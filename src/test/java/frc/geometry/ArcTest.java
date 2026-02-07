package frc.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.geometry.Arc;

public class ArcTest {
    @Test
    void arcTest() {
        Arc arc = new Arc(new Translation2d(100, 150), 10, Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(225));

        assertEquals(new Translation2d(90, 150), arc.nearestPointOnArc(new Translation2d(50, 150)));
        assertEquals(new Translation2d(90, 150), arc.nearestPointOnArc(new Translation2d(95, 150)));
        assertEquals(arc.getCenter().plus(new Translation2d(10, Rotation2d.fromDegrees(135))),
                arc.nearestPointOnArc(new Translation2d(110, 150)));
    }
}
