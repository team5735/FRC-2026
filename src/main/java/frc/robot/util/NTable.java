package frc.robot.util;

import java.util.Arrays;
import java.util.HashMap;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.ByteBuffer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import frc.robot.Robot;

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
 * NetworkTables is represented by exactly one NTable. This ensures that
 * each {@link GenericEntry} managed by every NTable is only created once,
 * allowing much more flexibility in client code. Additionally, this allows each
 * class to keep its own {@link #tablesToData} map instead of having it be
 * shared between all NTables.
 */
public class NTable {
    /** The root NTable, representing the root of all NetworkTables. */
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

    /** The subtables of this NTable. These are created on demand. */
    private HashMap<String, NTable> subs = new HashMap<>();

    /** The entries of this NTable. These are created on demand. */
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

    /** {@return the root NTable} */
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
     * <p>
     * Creates the subtable if it does not already exist.
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
     * Creates the entry if it does not already exist. Note that unless this entry
     * has been set, this function will see the topic as being of an unassigned
     * type. If the entry already exists with a different type than the one given in
     * type and warnOnWrongType is true, a warning will be printed.
     *
     * @param name            the name of the entry
     * @param type            the type of the entry
     * @param warnOnWrongType whether to warn if the entry has a type different from
     *                        the one specified
     */
    public GenericEntry getEntry(String name, NetworkTableType type, boolean warnOnWrongType) {
        boolean entryExisted = entries.containsKey(name);
        GenericEntry entry = entries.computeIfAbsent(name,
                n -> table.getTopic(n).getGenericEntry(type.getValueStr()));

        NetworkTableType entryType = entry.get().getType();
        if (warnOnWrongType && entryExisted && !entryType.equals(type)) {
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
     * Creates the entry if it does not already exist. Note that unless this entry
     * has been set, this function will see the topic as being of an unassigned
     * type. If the entry already exists with a different type than the one given in
     * type, a warning will be printed.
     *
     * @param name the name of the entry
     * @param type the type of the entry
     */
    public GenericEntry getEntry(String name, String type) {
        return getEntry(name, NetworkTableType.getFromString(type), true);
    }

    /**
     * Publishes a value of any accepted type to the NetworkTable.
     *
     * <p>
     * The type of the value is determined from the runtime type of the object. If
     * the type is not supported by NetworkTables, a warning is printed to
     * DriverStation and nothing is published. If the type differs from the type
     * previously posted to this entry in this NTable, a warning is printed and
     * nothing is published.
     *
     * <p>
     * Note that primitives such as `double` and `boolean` are automatically boxed
     * by Java into an {@link Object}, such as {@link Double} and {@link Boolean}.
     * Primitive arrays can also be passed without any necessary client-side code as
     * they are {@link Object}s.
     *
     * @param name  the name of the entry to publish
     * @param value the Object to publish
     */
    public void set(String name, Object value) {
        if (!NetworkTableEntry.isValidDataType(value)) {
            DriverStation.reportWarning("NTable entry " + table.getPath() + "/" + name
                    + " has invalid type; the passed object is of type " + value.getClass().getName(), true);
            return;
        }
        getEntry(name, NetworkTableType.getStringFromObject(value)).setValue(value);
    }

    /** A map of names to sent {@link Sendable}s. */
    private HashMap<String, Sendable> tablesToData = new HashMap<>();

    /**
     * Publishes a Sendable object to the NetworkTables.
     *
     * <p>
     * If the object has already been published (determined by object identity), it
     * will not be published again, as Sendables are automatically updated by
     * {@link #updateAllSendables()}, called in {@link Robot#robotPeriodic()}.
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

    private HashMap<String, Struct<?>> structs = new HashMap<>();

    public <T> void setStruct(String name, T value, Struct<T> struct) {
        NetworkTableInstance.getDefault().addSchema(struct);
        ByteBuffer buffer = StructBuffer.create(struct).write(value);
        int handle = getEntry(name, NetworkTableType.kRaw, true).getHandle();
        NetworkTablesJNI.setRaw(handle, NetworkTablesJNI.now(), buffer);
    }

    public <T> void setStruct(String name, T value) {
        if (structs.containsKey(name)) {
            // if the struct has already been extracted and cached by the rest of the
            // function, use it
            Struct<?> struct = structs.get(name);
            if (!struct.getTypeClass().isInstance(value)) {
                DriverStation.reportError(
                        "tried to publish a " + value.getClass().getName() + " object to " + name
                                + ", but a struct of type " + struct.getTypeClass().getName()
                                + " had already been registered for this entry in " + table.getPath(),
                        true);
                return;
            }
            @SuppressWarnings("unchecked")
            Struct<T> casted = (Struct<T>) struct;
            setStruct(name, value, casted);
            return;
        }

        // assuming that the struct is stored in T.struct, as is typical, get it using
        // runtime reflection (exciting!)
        Class<?> classType = value.getClass();
        try {
            Field field = classType.getField("struct");
            if (!Modifier.isStatic(field.getModifiers())) {
                DriverStation.reportWarning(
                        "could not find static field `struct` in class " + classType.getName() + " for entry " + name,
                        true);
                return;
            }
            Object possibleStruct = field.get(null);
            if (!(possibleStruct instanceof Struct<?> struct) ||
                    !struct.getTypeClass().isInstance(value)) {
                DriverStation.reportWarning(
                        "could not find static field `struct` of correct type in class " + classType.getName()
                                + " for entry " + name,
                        true);
                return;
            }
            @SuppressWarnings("unchecked")
            Struct<T> casted = (Struct<T>) struct;
            structs.put(name, struct);
            setStruct(name, value, casted);

        } catch (NoSuchFieldException e) {
            DriverStation.reportWarning(
                    "could not find static field `struct` in class " + classType.getName() + " for entry " + name,
                    e.getStackTrace());
        } catch (Exception e) {
            DriverStation.reportError(
                    "error retrieving static field `struct` in class " + classType.getName() + " for entry " + name,
                    e.getStackTrace());
        }
    }

    /**
     * Gets the value of the given type from the NetworkTable.
     *
     * <p>
     * If the requested type differs from the type retrieved from the entry, a
     * warning is printed and the current entry's type is returned.
     * 
     * @param name the name of the entry
     * @param type the type of the entry
     *
     * @return the requested value as a {@link NetworkTableValue}
     */
    public NetworkTableValue get(String name, NetworkTableType type) {
        return getEntry(name, type, false).get();
    }

    /** @see #get(String, NetworkTableType) */
    public double getDouble(String name) {
        return get(name, NetworkTableType.kDouble).getDouble();
    }

    /** @see #get(String, NetworkTableType) */
    public boolean getBoolean(String name) {
        return get(name, NetworkTableType.kBoolean).getBoolean();
    }

    /** @see #get(String, NetworkTableType) */
    public String getString(String name) {
        return get(name, NetworkTableType.kString).getString();
    }

    /** @see #get(String, NetworkTableType) */
    public long getInt(String name) {
        return get(name, NetworkTableType.kInteger).getInteger();
    }

    /** @see #get(String, NetworkTableType) */
    public float getLong(String name) {
        return get(name, NetworkTableType.kFloat).getFloat();
    }

    /** @see #get(String, NetworkTableType) */
    public byte[] getRaw(String name) {
        return get(name, NetworkTableType.kRaw).getRaw();
    }

    /** @see #get(String, NetworkTableType) */
    public double[] getDoubleArray(String name) {
        return get(name, NetworkTableType.kDoubleArray).getDoubleArray();
    }

    /** @see #get(String, NetworkTableType) */
    public boolean[] getBooleanArray(String name) {
        return get(name, NetworkTableType.kBooleanArray).getBooleanArray();
    }

    /** @see #get(String, NetworkTableType) */
    public String[] getStringArray(String name) {
        return get(name, NetworkTableType.kStringArray).getStringArray();
    }

    /** @see #get(String, NetworkTableType) */
    public long[] getIntArray(String name) {
        return get(name, NetworkTableType.kIntegerArray).getIntegerArray();
    }

    /** @see #get(String, NetworkTableType) */
    public float[] getFloatArray(String name) {
        return get(name, NetworkTableType.kFloatArray).getFloatArray();
    }

    /**
     * Attempts to retrieve and unpack a struct under the given path in
     * NetworkTables.
     *
     * <p>
     * If the given entry name does not currently store a raw value, this function
     * will error out like {@link #getRaw}. Further, if the value can be retrieved
     * but not unpacked, a warning is reported to the DriverStation.
     *
     * <p>
     * Note that this function is thoroughly untested as of now.
     *
     * @param name   the name of the entry
     * @param struct the struct by whose rules to unpack the data
     *
     * @return the unpacked struct
     */
    public <T> T getStruct(String name, Struct<T> struct) {
        byte[] raw = getRaw(name);
        if (raw.length == 0) {
            return null;
        }
        try {
            StructBuffer<T> buffer = StructBuffer.create(struct);
            return buffer.read(raw);
        } catch (RuntimeException e) {
            DriverStation.reportWarning("NTable entry " + table.getPath() +
                    "/" + name + " could not be unpacked: " + e.getMessage(), true);
            return null;
        }
    }

    /** {@return whether the given name is present in this NTable with that type} */
    public boolean isPresent(String name, NetworkTableType desiredType) {
        NetworkTableValue entry = getEntry(name, desiredType, false).get();
        return entry.getType().equals(desiredType);
    }

    /** {@return whether all of the given names are present in this NTable} */
    public boolean isPresent(String name) {
        return !getEntry(name, NetworkTableType.kRaw, false).get().getType().equals(NetworkTableType.kUnassigned);
    }

    /** {@return whether all of the given names are present in this NTable} */
    public boolean isPresent(String... names) {
        return Arrays.stream(names).allMatch(name -> isPresent(name));
    }

    /**
     * Makes the specified entries persist through program restarts.
     * 
     * @param names the names of the entries
     */
    public void makePersistent(String... names) {
        for (String name : names) {
            table.getEntry(name).setPersistent();
        }
    }

    /**
     * Makes the specified entries not persist through program restarts.
     * 
     * @param names the names of the entries
     */
    public void clearPersistent(String... names) {
        for (String name : names) {
            table.getEntry(name).clearPersistent();
        }
    }

    /**
     * Updates all sendables in the given table and its subtables.
     * 
     * <p>
     * Calling of this function is typically handled by
     * {@link Robot#robotPeriodic()}. It must be in such a function in order for
     * sendables that use the Sendable Property API to work.
     * 
     * @param table the table to update
     */
    public static void updateAllSendables(NTable table) {
        for (Sendable data : table.tablesToData.values()) {
            SendableRegistry.update(data);
        }
        for (NTable subtable : table.subs.values()) {
            updateAllSendables(subtable);
        }
    }

    /**
     * Updates all sendables.
     * 
     * <p>
     * Calling of this function is typically handled by
     * {@link Robot#robotPeriodic()}. It must be in such a function in order for
     * sendables that use the Sendable Property API to work.
     */
    public static void updateAllSendables() {
        updateAllSendables(root());
    }
}
