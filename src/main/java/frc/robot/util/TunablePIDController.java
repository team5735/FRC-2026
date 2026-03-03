package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

public class TunablePIDController {
    /** The wrapped PID controller. */
    protected PIDController controller;

    /** The table used to manage PID values. */
    private NTable table;

    /** Creates a new PID controller with zeroed defaults in /pid/{name}. */
    public TunablePIDController(String name) {
        this(NTable.root("pid"), name, 0, 0, 0);
    }

    /** Creates a new PID controller with zeroed defaults in {table}/{name}. */
    public TunablePIDController(NTable table, String name) {
        this.table = table.sub(name);
    }

    public void ensureP(double val) {
        this.table.ensure("kP");
    }

    public void ensureI(double val) {
        this.table.ensure("kI");
    }

    public void ensureD(double val) {
        this.table.ensure("kD");
    }

    public void ensurePID(double p, double i, double d) {
        ensureP(p);
        ensureI(i);
        ensureD(d);
    }

    public void ensureTolerance(double tolerance) {
        this.table.ensure("tolerance", tolerance);
    }

    boolean continuous = false;
    double continuousStart, continuousEnd;

    /**
     * Sets this PID controller to have a continuous input. This only takes effect
     * the next time setup() is called.
     */
    public void setContinuous(double start, double end) {
        continuous = true;
        continuousStart = start;
        continuousEnd = end;
    }

    /**
     * Sets this PID controller to have a non-continuous input. This only takes
     * effect the next time setup() is called.
     */
    public void unsetContinuous() {
        continuous = false;
    }

    /**
     * Set up this PID controller.
     *
     * <p>
     * This retrieves the PID constants from the table created during construction.
     * It also telemeterizes the setpoint and sets the controller to have the
     * specified setpoint and tolerance.
     *
     * @param setpoint
     * @param tolerance
     */
    public void setup(double setpoint) {
        controller = new PIDController(table.get("kP", 0), table.get("kI", 0), table.get("kD", 0));

        if (continuous) {
            controller.enableContinuousInput(continuousStart, continuousEnd);
        }

        controller.setSetpoint(setpoint);
        table.set("setpoint", setpoint);
        controller.setTolerance(table.get("tolerance", 0));
        table.set("tolerance", table.get("tolerance", 0));
    }

    /** Resets the PID controller. */
    public void reset() {
        controller.reset();
    }

    /**
     * Re-fetches the PID constants from the table created during construction and
     * applies them to the PID controller.
     */
    public void refetch() {
        controller.setPID(table.get("kP", 0), table.get("kI", 0), table.get("kD", 0));
    }

    /**
     * Sets the PID constants.
     *
     * @param p kP
     * @param i kI
     * @param d kD
     */
    public void setPID(double p, double i, double d) {
        controller.setPID(p, i, d);
        table.set("kP", p);
        table.set("kI", i);
        table.set("kD", d);
    }

    /**
     * Returns the next output of the PID controller.
     * 
     * <p>
     * Also telemeterizes the given and PID controller output.
     * 
     * @param measurement
     * @return The controller output, or zero if atSetpoint.
     */
    public double calculate(double measurement) {
        double value = controller.calculate(measurement);
        table.set("measurement", measurement);
        table.set("output", value);
        table.set("error", controller.getError());
        return value;
    }

    /** {@return the next output of the PID controller} */
    public double calculate(double measurement, double setpoint) {
        double value = controller.calculate(measurement, setpoint);
        table.set("measurement", measurement);
        table.set("output", value);
        table.set("error", controller.getError());
        table.set("setpoint", setpoint);
        return value;
    }

    /** {@return whether this PIDController is within tolerance of the setpoint} */
    public boolean atSetpoint() {
        return this.controller.atSetpoint();
    }

    /** {@return the wrapped PIDController} */
    public PIDController getController() {
        return controller;
    }
}
