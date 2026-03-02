package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

/**
 * TunableNumber allows for runtime tuning of numeric values via NetworkTables.
 * Gets value from dashboard in tuning mode, returns default otherwise.
 */
public class TunableNumber extends Number{
    private String tableKey = "TunableNumbers";

    private String key;
    private double defaultValue;
    private double lastHasChangedValue = defaultValue;

    private DoubleEntry entry;


    /**
     * Create a new TunableNumber with the default value
     * 
     * @param name Key on dashboard
     * @param defaultValue Default value
     */
    public TunableNumber(String name, double defaultValue) {
        this.key = tableKey + "/" + name;
        setDefault(defaultValue);
    }

    /**
     * Create a new TunableNumber with the default value
     * 
     * @param name Key on dashboard
     * @param defaultValue Default value
     * @param tableKey Key for the table to put the number in
     */
    public TunableNumber(String name, double defaultValue, String tableKey) {
        this.tableKey = tableKey;
        this.key = tableKey + "/" + name;
        setDefault(defaultValue);
    }

    /**
     * Get the default value for the number that has been set
     * 
     * @return The default value
     */
    public double getDefault() {
        return defaultValue;
    }

    /**
     * Set the default value of the number
     * 
     * @param defaultValue The default value
     */
    private void setDefault(double defaultValue) {
        this.defaultValue = defaultValue;
        if (Constants.tuningMode) {
            entry = NetworkTableInstance.getDefault().getTable(tableKey).getDoubleTopic(key).getEntry(defaultValue);
            entry.accept(defaultValue);
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode
     * 
     * @return The current value
     */
    private double get() {
        return Constants.tuningMode ? entry.get()
                : defaultValue;
    }

    /**
     * Checks whether the number has changed since our last check
     * 
     * @return True if the number has changed since the last time this method was called, false
     *                 otherwise
     */
    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastHasChangedValue) {
            lastHasChangedValue = currentValue;
            return true;
        }

        return false;
    }

    @Override
    public int intValue() {
        return (int) get();
    }
    @Override
    public long longValue() {
        return (long) get();
    }
    @Override
    public float floatValue() {
        return (float) get();
    }
    @Override
    public double doubleValue() {
        return get();
    }
    @Override
    public String toString() {
        return String.valueOf(get());
    }
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof TunableNumber) {
            return ((TunableNumber) obj).get() == get();
        }
        return false;
    }
    @Override
    public int hashCode() {
        return Double.hashCode(get());
    }
}