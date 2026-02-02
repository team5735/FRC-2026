package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;

public class NTable {
    private static NTable root;

    private NetworkTable table;

    private HashMap<String, NTable> subs = new HashMap<>();
    private HashMap<String, GenericEntry> entries = new HashMap<>();

    private NTable(NetworkTable table) {
        System.out.println("creating NTable for " + table.getPath());
        this.table = table;
    }

    public static NTable root() {
        if (root == null) {
            root = new NTable(NetworkTableInstance.getDefault().getTable(""));
        }
        return root;
    }

    public NTable sub(String name) {
        return subs.computeIfAbsent(name, n -> new NTable(table.getSubTable(n)));
    }

    public void set(String name, Object value) {
        String type = NetworkTableType.getStringFromObject(value);
        GenericEntry entry = entries.computeIfAbsent(name, n -> table.getTopic(n).getGenericEntry(type));
        boolean success = entry.setValue(value);
        if (!success) {
            System.out.println("warning: NTable entry " + table.getPath() + "/" + name + " was not a " + type);
        }
    }

    public NetworkTableValue get(String name, Object value) {
        String type = NetworkTableType.getStringFromObject(value);
        GenericEntry entry = entries.computeIfAbsent(name, n -> table.getTopic(n).getGenericEntry(type));
        return entry.get();
    }
}
