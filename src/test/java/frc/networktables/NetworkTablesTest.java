package frc.networktables;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import frc.robot.util.NTable;

public class NetworkTablesTest {
    @Test
    void singletonsTest() {
        Assertions.assertTrue(NTable.root("test") == NTable.root("test"));

        NTable testTable = NTable.root("test");
        Assertions.assertTrue(testTable == NTable.root("test"));
        Assertions.assertTrue(testTable.sub("sub") == testTable.sub("sub"));
        testTable.set("ent", true);
        Assertions.assertTrue(testTable.getEntry("ent", NetworkTableType.kBoolean) == testTable.getEntry("ent",
                NetworkTableType.kBoolean));
    }

    @Test
    void basicNTTest() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("");
        DoublePublisher pub = table.getDoubleTopic("test").publish();
        pub.set(1.0);
        Assertions.assertEquals(1.0, table.getDoubleTopic("test").subscribe(0.0).get());
    }

    public static void main(String[] args) {
        NetworkTablesTest test = new NetworkTablesTest();
        test.singletonsTest();
    }
}
