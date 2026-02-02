package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    /**
     * {@return the NetworkTable this NTable represents}
     */
    public NetworkTable getTable() {
        return table;
    }

    /**
     * The subtables of this NTable. These are created on demand.
     */
    private HashMap<String, NTable> subs = new HashMap<>();

    /**
     * The entries of this NTable. These are created on demand.
     */
    private HashMap<String, GenericEntry> entries = new HashMap<>();

    /**
     * Creates a new NTable.
     *
     * @param table the NetworkTable this NTable will represent
     */
    private NTable(NetworkTable table) {
        this.table = table;
    }

    /**
     * {@return the root NTable}
     */
    public static NTable root() {
        return root;
    }

    /**
     * {@return a subtable of the root NTable}
     * 
     * @param name the name of the subtable
     */
    public static NTable root(String name) {
        return root().sub(name);
    }

    /**
     * {@return a subtable of this NTable}
     * 
     * @param name the name of the subtable
     */
    public NTable sub(String name) {
        return subs.computeIfAbsent(name, n -> new NTable(table.getSubTable(n)));
    }

    /**
     * {@return an entry of this NTable}
     *
     * <p>
     * Adds the entry to {@link #entries} if it's not already present.
     *
     * @param name the name of the entry
     * @param type the type of the entry
     */
    public GenericEntry getEntry(String name, NetworkTableType type) {
        return entries.computeIfAbsent(name, key -> table.getTopic(key).getGenericEntry(type.getValueStr()));
    }

    /**
     * {@return an entry of this NTable}
     *
     * <p>
     * Adds the entry to {@link #entries} if it's not already present.
     *
     * @param name the name of the entry
     * @param type the type of the entry
     */
    public GenericEntry getEntry(String name, String type) {
        return getEntry(name, NetworkTableType.getFromString(type));
    }

    /**
     * Publishes a value of any accepted type to the NetworkTable.
     *
     * <p>
     * The type of the value is inferred from the type of the object. If the type is
     * not supported by NetworkTables, a warning is printed to DriverStation and
     * nothing is published. If the type differs from the type previously posted to
     * this entry in this NTable, a warning is printed and nothing is published.
     *
     * <p>
     * Note that primitives such as `double` and `boolean` are automatically boxed
     * by Java into an {@link Object}, such as {@link Double} and {@link Boolean}.
     * Primitive arrays can also be passed without issue as they are
     * {@link Object}s.
     *
     * @param name  the name of the entry.
     * @param value the value to publish.
     */
    public void set(String name, Object value) {
        if (!NetworkTableEntry.isValidDataType(value)) {
            DriverStation.reportWarning("NTable entry " + table.getPath() + "/" + name
                    + " has invalid type; the passed object is of type " + value.getClass().getName(), true);
            return;
        }
        GenericEntry entry = getEntry(name, NetworkTableType.getStringFromObject(value));
        boolean success = entry.setValue(value);
        if (!success) {
            DriverStation.reportWarning(
                    "NTable entry " + table.getPath() + "/" + name + " was not a "
                            + NetworkTableType.getStringFromObject(value),
                    true);
        }
    }

    /**
     * {@return the value of an entry of any type}
     * 
     * @param name the name of the entry
     * @param type the type of the entry
     */
    public NetworkTableValue get(String name, NetworkTableType type) {
        return getEntry(name, type).get();
    }

    /**
     * Publishes a Sendable object to the NetworkTables.
     *
     * <p>
     * This behaves pretty much identically to
     * {@link SmartDashboard#putData(String, Sendable)}.
     * 
     * @param name the name of the entry
     * @param data the object to publish
     */
    public void setSendable(String name, Sendable data) {
        NTable dataTable = sub(name);
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(dataTable.getTable());
        SendableRegistry.publish(data, builder);
        builder.startListeners();
        dataTable.set(".name", name);
    }

    /**
     * Makes the specified entries persist through program restarts.
     * 
     * @param name the name of the entry
     */
    public void makePersistent(String... names) {
        for (String name : names) {
            table.getEntry(name).setPersistent();
        }
    }

    /**
     * Makes the specified entry not persist through program restarts.
     * 
     * @param name the name of the entry
     */
    public void clearPersistent(String... names) {
        for (String name : names) {
            table.getEntry(name).clearPersistent();
        }
    }
}
