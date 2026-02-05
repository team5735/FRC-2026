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
    private static final NTable root = new NTable(NetworkTableInstance.getDefault().getTable(""), null);

    /**
     * The NetworkTable this NTable represents. This NTable object can be seen as a
     * wrapper around its table.
     */
    private final NetworkTable table;

    private final NTable parent;

    /** {@return the parent of this NTable, or null if it is root} */
    public NTable getParent() {
        return parent;
    }

    /** {@return the NetworkTable this NTable represents} */
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
    private NTable(NetworkTable table, NTable parent) {
        this.table = table;
        this.parent = parent;
        System.out.println("created ntable for " + table.getPath());
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
        return subs.computeIfAbsent(name, n -> new NTable(table.getSubTable(n), this));
    }

    /**
     * {@return an entry of this NTable}
     *
     * <p>
     * Adds the entry to {@link #entries} if it's not already present.
     *
     * <p>
     * Note that unless this entry has been set, this function will see the topic as
     * being of an unassigned type.
     *
     * @param name the name of the entry
     * @param type the type of the entry
     */
    public GenericEntry getEntry(String name, NetworkTableType type) {
        boolean entryExisted = entries.containsKey(name);
        GenericEntry entry = entries.computeIfAbsent(name,
                n -> table.getTopic(n).getGenericEntry(type.getValueStr()));

        NetworkTableType entryType = entry.get().getType();
        if (entryExisted && !entryType.equals(type)) {
            DriverStation.reportWarning(
                    "NTable entry " + table.getPath() + "/" + name + " had a type different from '"
                            + type + "'; its type was '" + entryType + "'",
                    false);
        }

        return entry;
    }

    /**
     * {@return an entry of this NTable}
     *
     * <p>
     * Adds the entry to {@link #entries} if it's not already present.
     *
     * <p>
     * Note that unless this entry has been set, this function will see the topic as
     * being of an unassigned type.
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
        NetworkTableValue value = getEntry(name, type).get();
        if (!value.getType().equals(type)) {
            DriverStation.reportWarning(
                    "NTable entry " + table.getPath() + "/" + name + " was not a "
                            + type.getValueStr(),
                    true);
        }
        return value;
    }

    /** {@return the requested double} */
    public double getDouble(String name) {
        return get(name, NetworkTableType.kDouble).getDouble();
    }

    /** {@return the requested boolean} */
    public boolean getBoolean(String name) {
        return get(name, NetworkTableType.kBoolean).getBoolean();
    }

    /** {@return the requested string} */
    public String getString(String name) {
        return get(name, NetworkTableType.kString).getString();
    }

    /** {@return the requested int} */
    public long getInt(String name) {
        return get(name, NetworkTableType.kInteger).getInteger();
    }

    /** {@return the requested float} */
    public float getLong(String name) {
        return get(name, NetworkTableType.kFloat).getFloat();
    }

    /** {@return the requested raw bytes} */
    public byte[] getRaw(String name) {
        return get(name, NetworkTableType.kRaw).getRaw();
    }

    /** {@return the requested array of doubles} */
    public double[] getDoubleArray(String name) {
        return get(name, NetworkTableType.kDoubleArray).getDoubleArray();
    }

    /** {@return the requested array of booleans} */
    public boolean[] getBooleanArray(String name) {
        return get(name, NetworkTableType.kBooleanArray).getBooleanArray();
    }

    /** {@return the requested array of strings} */
    public String[] getStringArray(String name) {
        return get(name, NetworkTableType.kStringArray).getStringArray();
    }

    /** {@return the requested array of ints} */
    public long[] getIntArray(String name) {
        return get(name, NetworkTableType.kIntegerArray).getIntegerArray();
    }

    /** {@return the requested array of floats} */
    public float[] getFloatArray(String name) {
        return get(name, NetworkTableType.kFloatArray).getFloatArray();
    }

    /** A map of names to sent {@link Sendable}s. */
    HashMap<String, Sendable> tablesToData = new HashMap<>();

    /**
     * Publishes a Sendable object to the NetworkTables.
     *
     * <p>
     * If the object has already been published, it will not be published again, as
     * Sendables are automatically updated by the robot loop.
     * 
     * @param name the name of the entry
     * @param data the object to publish
     */
    public void setSendable(String name, Sendable data) {
        if (tablesToData.get(name) == data) {
            // this sendable has already been published and will automatically update
            return;
        }
        tablesToData.put(name, data);
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(sub(name).getTable());
        SendableRegistry.publish(data, builder);
        builder.startListeners();
        sub(name).set(".name", name);
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

    private static void updateAllSendables(NTable table) {
        for (Sendable data : table.tablesToData.values()) {
            SendableRegistry.update(data);
        }
        for (NTable subtable : table.subs.values()) {
            updateAllSendables(subtable);
        }
    }

    public static void updateAllSendables() {
        updateAllSendables(root());
    }
}
