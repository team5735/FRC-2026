package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

/**
 * A {@link NetworkTable}.
 *
 * <p>
 * This class represents a NetworkTable. Its purpose is to make the
 * NetworkTables API nicer to use, as well as adding consistent caching logic
 * for entries and tables to hopefully permanently eliminate continuously making
 * publishers or tables.
 *
 * <p>
 * Note that one invariant of this class is that every table of the
 * NetworkTables is represented by exactly one NTable. This ensures that every
 * NTable is effectively a singleton of sorts. Additionally, ths ensures that
 * each {@link GenericEntry} managed by every NTable is only created once,
 * allowing much more flexibility in client code.
 */
public class NTable {
    /**
     * The root NTable, representing the root of all NetworkTables.
     */
    private static final NTable root = new NTable(NetworkTableInstance.getDefault().getTable(""));

    /**
     * The NetworkTable this NTable represents. This NTable object can be seen as a
     * wrapper around its table.
     */
    private final NetworkTable table;

    public NetworkTable getTable() {
        return table;
    }

    private HashMap<String, NTable> subs = new HashMap<>();
    private HashMap<String, GenericEntry> entries = new HashMap<>();

    private NTable(NetworkTable table) {
        System.out.println("creating NTable for " + table.getPath());
        this.table = table;
    }

    public static NTable root() {
        return root;
    }

    public static NTable root(String name) {
        return root().sub(name);
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

    public NetworkTableValue get(String name, NetworkTableType type) {
        GenericEntry entry = entries.computeIfAbsent(name, n -> table.getTopic(n).getGenericEntry(type.getValueStr()));
        return entry.get();
    }

    public void setSendable(String name, Sendable data) {
        NTable dataTable = sub(name);
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(dataTable.getTable());
        SendableRegistry.publish(data, builder);
        builder.startListeners();
        dataTable.set(".name", name);
    }
}
