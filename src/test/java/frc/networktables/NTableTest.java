package frc.networktables;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.networktables.NetworkTableType;
import frc.robot.util.NTable;

public class NTableTest {
    @Test
    void singletonsTest() throws Exception {
        assertTrue(NTable.root("test") == NTable.root("test"));

        NTable testTable = NTable.root("test");
        assertTrue(testTable == NTable.root("test"));
        assertTrue(testTable.sub("sub") == testTable.sub("sub"));
        assertTrue(testTable.getEntry("ent", NetworkTableType.kBoolean) == testTable.getEntry("ent",
                NetworkTableType.kBoolean));
    }
}
