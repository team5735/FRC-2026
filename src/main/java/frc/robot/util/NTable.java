package frc.robot.util;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.HashMap;

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
 * each {@link NetworkTableEntry} managed by every NTable is only created once,
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
    private HashMap<String, NetworkTableEntry> entries = new HashMap<>();

    /**
     * Creates a new NTable.
     *
     * @param table the NetworkTable this NTable will represent
     */
    private NTable(NetworkTable table, NTable parent) {
        this.table = table;
        this.parent = parent;
        long depth = this.table.getPath().chars().filter(c -> c == '/').count();
        if (depth > 50) {
            DriverStation.reportWarning("very long NTable of depth " + depth
                    + " created! be careful! its path is " + this.table.getPath(), true);
        }

        for (String entry : this.table.getKeys()) {
            entries.put(entry, this.table.getEntry(entry));
        }
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
     * @param name the name of the entry
     */
    public NetworkTableEntry getEntry(String name) {
        return entries.computeIfAbsent(name, n -> table.getEntry(n));
    }

    /**
     * Publishes a value of any accepted type to the NetworkTable.
     *
     * <p>
     * The type of the value is determined from the runtime type of the object
     * passed to the parameter {@code value}. This function acts as a catch-all for
     * anything that can be posted to NetworkTables, in the spirit of this class's
     * 'set and forget' philosophy.
     * 
     * <p>
     * If the type is a so-called 'simple' type, it is passed to
     * {@link #setSimple(String, Object)}. See the end of this function's
     * documentation for a discussion of the term 'simple'.
     * 
     * <p>
     * If the type is a {@link Sendable}, it is passed to
     * {@link #setSendable(String, Sendable)}.
     *
     * <p>
     * Otherwise, the type is checked to see whether it has a registered
     * {@code Struct<T>} associated with its class type.
     * If so, it is passed to {@link #setStruct(String, Struct)}.
     * set also attempts to automatically detect if the passed object's class type
     * has a static member named 'struct', and if so, that struct is registered for
     * the passed class type and the object is then passed to
     * {@link #setStruct(String, Struct)}.
     *
     * <p>
     * If the object is of array type, it is checked whether it is an array of
     * struct-serializable objects. If that is the case, then it is serialized as an
     * array of structs.
     *
     * <p>
     * If none of the above cases apply, a warning is printed and the object is
     * ignored.
     *
     * <p>
     * The following types are considered 'simple':
     * 
     * <ul>
     * <li>{@code Boolean}</li>
     * <li>{@code Float}</li>
     * <li>{@code Long}</li>
     * <li>{@code Double}</li>
     * <li>{@code String}</li>
     * </ul>
     * 
     * Arrays of any of the above, as well as arrays of their respective primitive
     * types (except for String, which does not have an associated primitive type)
     * are also considered 'simple'. For example, the following types are 'simple':
     * {@code String[]}, {@code double[]}, {@code int}, etc. Note that primitive
     * types such as {@code int} automatically get 'boxed' into their corresponding
     * {@link Object} type, such as {@link Integer} in the case of int. Therefore,
     * passing a primitive to the value parameter of this function will work as
     * expected.
     * 
     * <p>
     * There are two other types considered 'simple': {@code byte[]} and
     * {@code Byte[]}. These are classified as 'raw' data and are typically used to
     * send structs across NetworkTables (see {@link #setStruct(String, Struct)}).
     *
     * <p>
     * Furthermore, any numeric primitive box type that extends {@code Number} is
     * also considered primitive and will be sent as a double, except for the
     * non-double numerics mentioned above, {@code Float} and {@code Long}. For
     * example, this function will accept a {@code Byte} (not an array of byte;
     * that would fall under the previous paragraph!) and send it as a double.
     * 
     * @param name  the name of the entry to publish
     * @param value the Object to publish
     *
     * @bug This function does not properly handle classes that are only
     *      de/serializable with Protobuffers.
     */
    public <T> void set(String name, T value) {
        // If the value is a simple type, publish it simply.
        if (NetworkTableEntry.isValidDataType(value)) {
            setSimple(name, value);
            return;
        }

        // If the value is a Sendable, use setSendable().
        if (value instanceof Sendable casted) {
            setSendable(name, casted);
            return;
        }

        // getStructForObject returns null when the object doesn't have an associated
        // struct for its class. Using this, we can verify that the object is a struct
        // type.
        Struct<?> possibleStruct = getStructForObject(value.getClass());
        if (possibleStruct != null) {
            @SuppressWarnings("unchecked")
            Struct<T> casted = (Struct<T>) possibleStruct;
            setStruct(name, value, casted);
            return;
        }

        // If the object is an array, check whether it is an array of
        // struct-serializable
        // objects.
        if (value.getClass().isArray() && value instanceof Object[] casted) {
            possibleStruct = getStructForObject(casted.getClass().getComponentType());
            if (possibleStruct != null) {
                // If we got this far, we know that castedStruct is Struct<T> where T is the
                // component type of value. Due to Java generics being a pile of type-erasing
                // bullshit, we can cast everything to be in terms of Object and it should just
                // work.
                @SuppressWarnings("unchecked")
                Struct<Object> castedStruct = (Struct<Object>) possibleStruct;
                setStructs(name, casted, castedStruct);
                return;
            }
        }

        // If none of the above cases apply, print a warning.
        DriverStation.reportError(
                "NTable: Could not publish value of type " + value.getClass().getName() + " to entry " + name
                        + ": it is not supported.",
                false);
    }

    /**
     * Publishes a value of any accepted 'simple' type to the NetworkTable.
     *
     * <p>
     * See the documentation for {@link #set(String, Object)} for a discussion of
     * 'simple' types.
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
    public void setSimple(String name, Object value) {
        if (!NetworkTableEntry.isValidDataType(value)) {
            DriverStation.reportWarning("NTable entry " + table.getPath() + "/" + name
                    + " has invalid type; the passed object is of type " + value.getClass().getName(), true);
            return;
        }
        getEntry(name).setValue(value);
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

    /** Publishes a ByteBuffer to the NetworkTable. */
    private void publishRawBuffer(String name, ByteBuffer buffer) {
        NetworkTablesJNI.setRaw(getEntry(name).getHandle(), NetworkTablesJNI.now(), buffer);
    }

    /** A map of class types to registered struct objects. */
    private HashMap<Class<?>, Struct<?>> cachedStructs = new HashMap<>();

    /**
     * Uses runtime reflection on classType to retrieve its static 'struct' member,
     * if it exists.
     *
     * <p>
     * If in any case the associated struct for this class type cannot be resolved,
     * null is returned. This function also caches results it resolves to avoid
     * repeated reflection. The cache key is the passed class type. See
     * {@link #cachedStructs} for the cache.
     *
     * @param classType the class type to get the struct for
     * 
     * @return the struct for the given class type
     */
    private <T> Struct<T> getStructForObject(Class<?> classType) {
        if (classType == null) {
            DriverStation.reportWarning("null class type passed trying to retrieve struct", true);
            return null;
        }

        if (cachedStructs.containsKey(classType)) {
            // if the struct has already been extracted and cached by the rest of the
            // function, use it
            Struct<?> struct = cachedStructs.get(classType);
            if (!struct.getTypeClass().isAssignableFrom(classType)) {
                DriverStation.reportError(
                        "tried to publish a " + classType.getName() + ", but a struct of type "
                                + struct.getTypeClass().getName() + " had already been registered for this entry in "
                                + table.getPath(),
                        true);
                return null;
            }
            @SuppressWarnings("unchecked")
            Struct<T> casted = (Struct<T>) struct;
            return casted;
        }

        // assuming that the struct is stored in T.struct, as is the convention, get it
        // using runtime reflection (exciting!)
        try {
            Field field = classType.getField("struct");
            if (!Modifier.isStatic(field.getModifiers())) {
                return null;
            }
            Object possibleStruct = field.get(null);
            if (!(possibleStruct instanceof Struct<?> struct) || !struct.getTypeClass().isAssignableFrom(classType)) {
                return null;
            }
            @SuppressWarnings("unchecked")
            Struct<T> casted = (Struct<T>) struct;
            cachedStructs.put(classType, struct);
            return casted;

        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Publishes a struct to the NetworkTable.
     *
     * <p>
     * This function uses its struct parameter to serialize the passed value and
     * then publishes the raw value to NetworkTables as a 'raw' entry.
     *
     * @param name   the name of the entry to publish
     * @param value  the value to publish
     * @param struct the struct with which to serialize the value
     */
    public <T> void setStruct(String name, T value, Struct<T> struct) {
        NetworkTableInstance.getDefault().addSchema(struct);
        publishRawBuffer(name, StructBuffer.create(struct).write(value));
    }

    /**
     * Publishes an array of structs to the NetworkTable.
     *
     * <p>
     * This function uses its struct parameter to serialize the passed value and
     * then publishes the raw value to NetworkTables as a 'raw' entry.
     *
     * @param name   the name of the entry to publish
     * @param values the values to publish
     * @param struct the struct with which to serialize the values
     */
    public <T> void setStructs(String name, T[] values, Struct<T> struct) {
        NetworkTableInstance.getDefault().addSchema(struct);
        publishRawBuffer(name, StructBuffer.create(struct).writeArray(values));
    }

    /**
     * Publishes a struct to the NetworkTable.
     *
     * <p>
     * This function uses runtime reflection to try and figure out the right
     * {@code Struct<T>} object to use to serialize the passed value. It expects to
     * find this as a static member {@code T.value}.
     *
     * @param name  the name of the entry to publish
     * @param value the value to publish
     */
    public <T> void setStruct(String name, T value) {
        Struct<T> struct = getStructForObject(value.getClass());
        if (struct != null) {
            setStruct(name, value, struct);
        }
    }

    /**
     * Publishes an array of structs to the NetworkTable.
     *
     * <p>
     * This function uses runtime reflection to try and figure out the right
     * {@code Struct<T>} object to use to serialize the passed value. It expects to
     * find this as a static member {@code T.value}.
     *
     * @param name   the name of the entry to publish
     * @param values the values to publish
     */
    public <T> void setStructs(String name, T[] values) {
        Struct<T> struct = getStructForObject(values.getClass().getComponentType());
        if (struct != null) {
            setStructs(name, values, struct);
        }
    }

    /**
     * Retrieves a value of any accepted type from NetworkTables.
     *
     * <p>
     * The type of the value is determined from the runtime type of the object
     * passed to the parameter {@code defaultValue}. This function acts as a
     * catch-all for anything that can be retrieved from NetworkTables, in the
     * spirit of this class's 'set and forget' philosophy.
     * 
     * <p>
     * If the type is a so-called 'simple' type, this function defers to
     * {@link #getSimple(String, NetworkTableType)}. See the end of this function's
     * documentation for a discussion of the term 'simple'.
     * 
     * <p>
     * If the type is a {@link Sendable}, this function returns the stored sendable
     * as was last set under this name. In other words, setting a sendable then
     * getting it should return the exact same object back to the caller.
     *
     * <p>
     * Otherwise, the type is checked to see whether it has a registered
     * {@code Struct<T>} associated with its class type.
     * If so, this function defers to {@link #getStruct(String, Struct)}.
     * set also attempts to automatically detect if the passed object's class type
     * has a static member named 'struct', and if so, that struct is registered for
     * the passed class type and the function then defers to
     * {@link #getStruct(String, Struct)}.
     *
     * <p>
     * If the object is of array type, it is checked whether it is an array of
     * struct-serializable objects. If that is the case, then it is deserialized as
     * an array of structs by deferring to {@link #getStructs(String, Struct)}.
     *
     * <p>
     * If none of the above cases apply, a warning is printed and the defalut value
     * is returned.
     * 
     * <p>
     * The following types are considered 'simple':
     * 
     * <ul>
     * <li>{@code Boolean}</li>
     * <li>{@code Float}</li>
     * <li>{@code Long}</li>
     * <li>{@code Double}</li>
     * <li>{@code String}</li>
     * </ul>
     * 
     * Arrays of any of the above, as well as arrays of their respective primitive
     * types (except for String, which does not have an associated primitive type)
     * are also considered 'simple'. For example, the following types are 'simple':
     * {@code String[]}, {@code double[]}, {@code int}, etc. Note that primitive
     * types such as {@code int} automatically get 'boxed' into their corresponding
     * {@link Object} type, such as {@link Integer} in the case of int. Therefore,
     * passing a primitive to the value parameter of this function will work as
     * expected.
     * 
     * <p>
     * There are two other types considered 'simple': {@code byte[]} and
     * {@code Byte[]}. These are classified as 'raw' data and are typically used to
     * send structs across NetworkTables (see {@link #setStruct(String, Struct)}).
     *
     * <p>
     * Furthermore, any numeric primitive box type that extends {@code Number} is
     * also considered primitive and will be sent as a double, except for the
     * non-double numerics mentioned above, {@code Float} and {@code Long}. For
     * example, this function will accept a {@code Byte} (not an array of byte;
     * that would fall under the previous paragraph!) and send it as a double.
     * 
     * @param name         the name of the entry to retrieve from
     * @param defaultValue the default value to return if the entry does not exist
     *                     or is invalid
     *
     * @bug This function does not properly handle classes that are only
     *      de/serializable with Protobuffers.
     */
    public <T> T get(String name, T defaultValue) {
        // If the value is a simple type, retrieve it and attempt to cast it to the
        // requested type.
        if (NetworkTableEntry.isValidDataType(defaultValue)) {
            NetworkTableValue retrieved = getEntry(name).getValue();
            Class<?> classType = defaultValue.getClass();
            if (!classType.isInstance(retrieved.getValue())) {
                return defaultValue;
            }
            @SuppressWarnings("unchecked")
            T value = (T) defaultValue.getClass().cast(retrieved.getValue());
            return value;
        }

        // If the value is a Sendable, it should be in the tablesToData map; otherwise
        // you'll get a null, which is probably fine in terms of interface design.
        if (defaultValue instanceof Sendable) {
            @SuppressWarnings("unchecked")
            T value = (T) tablesToData.get(name);
            return value;
        }

        // getStructForObject returns null when the object doesn't have an associated
        // struct for its class. Using this, we can verify that the object is a struct
        // type.
        Struct<?> possibleStruct = getStructForObject(defaultValue.getClass());
        if (possibleStruct != null) {
            @SuppressWarnings("unchecked")
            Struct<T> casted = (Struct<T>) possibleStruct;
            T result = getStruct(name, casted);
            if (result == null) {
                return defaultValue;
            }
            return result;
        }

        // If the object is an array, check whether it is an array of
        // struct-deserializable objects.
        if (defaultValue.getClass().isArray() && defaultValue instanceof Object[] casted) {
            Struct<?> possibleStruct2 = getStructForObject(casted.getClass().getComponentType());
            if (possibleStruct2 != null) {
                // We already know that T is an array type, so it's safe to assume that and cast
                // directry to it. Note that the 'T' in getStructs is different from the T here,
                // and is in fact the value-type of the T here.
                @SuppressWarnings("unchecked")
                T result = (T) getStructs(name, possibleStruct2);
                if (result == null) {
                    return defaultValue;
                }
                return result;
            }
        }

        // If none of the above cases apply, print a warning.
        DriverStation.reportError(
                "NTable: Could not retrieve value of type " + defaultValue.getClass().getName() + " to entry " + name
                        + ": it is not supported.",
                true);
        return defaultValue;
    }

    /**
     * Gets the value of the given type from the NetworkTable.
     *
     * <p>
     * If the requested type differs from the type retrieved from the entry, a
     * warning is printed and a value of the current entry's type is returned. If no
     * value has been published under this name, returns a NetworkTableValue with
     * type kUnassigned.
     * 
     * @param name the name of the entry
     * @param type the type of the entry
     *
     * @return the requested value as a {@link NetworkTableValue}
     */
    public NetworkTableValue getSimple(String name, NetworkTableType type) {
        return getEntry(name).getValue();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public double getDouble(String name) {
        return getSimple(name, NetworkTableType.kDouble).getDouble();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public boolean getBoolean(String name) {
        return getSimple(name, NetworkTableType.kBoolean).getBoolean();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public String getString(String name) {
        return getSimple(name, NetworkTableType.kString).getString();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public long getInt(String name) {
        return getSimple(name, NetworkTableType.kInteger).getInteger();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public float getLong(String name) {
        return getSimple(name, NetworkTableType.kFloat).getFloat();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public byte[] getRaw(String name) {
        return getSimple(name, NetworkTableType.kRaw).getRaw();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public double[] getDoubleArray(String name) {
        return getSimple(name, NetworkTableType.kDoubleArray).getDoubleArray();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public boolean[] getBooleanArray(String name) {
        return getSimple(name, NetworkTableType.kBooleanArray).getBooleanArray();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public String[] getStringArray(String name) {
        return getSimple(name, NetworkTableType.kStringArray).getStringArray();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public long[] getIntArray(String name) {
        return getSimple(name, NetworkTableType.kIntegerArray).getIntegerArray();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public float[] getFloatArray(String name) {
        return getSimple(name, NetworkTableType.kFloatArray).getFloatArray();
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

    /**
     * Attempts to retrieve and unpack an array of structs under the given path in
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
    public <T> T[] getStructs(String name, Struct<T> struct) {
        byte[] raw = getRaw(name);
        if (raw.length == 0) {
            return null;
        }
        try {
            StructBuffer<T> buffer = StructBuffer.create(struct);
            return buffer.readArray(raw);
        } catch (RuntimeException e) {
            DriverStation.reportWarning("NTable entry " + table.getPath() +
                    "/" + name + " could not be unpacked: " + e.getMessage(), true);
            return null;
        }
    }

    /** {@return whether the given name is present in this NTable with that type} */
    public boolean existsAs(String name, NetworkTableType desiredType) {
        return getEntry(name).getType().equals(desiredType);
    }

    /** {@return whether the given name is present in this NTable} */
    public boolean exists(String name) {
        return getEntry(name).exists();
    }

    /** {@return whether all of the given names are present in this NTable} */
    public boolean exists(String... names) {
        return Arrays.stream(names).allMatch(name -> exists(name));
    }

    /**
     * {@return whether the given name is present in this NTable, and if not, sets
     * it to the passed value}
     */
    public <T> boolean ensure(String name, T value) {
        if (!exists(name)) {
            set(name, value);
            return true;
        }
        return false;
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
