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

    /** Creates a new PID controller with the specified defaults in /pid/{name}. */
    public TunablePIDController(String name, double _p, double _i, double _d) {
        this(NTable.root("pid"), name, _p, _i, _d);
    }

    /** Creates a new PID controller with zeroed defaults in {table}/{name}. */
    public TunablePIDController(NTable table, String name) {
        this(table, name, 0, 0, 0);
    }

    /** Creates a new PID controller with the specified values in {table}/{name}. */
    public TunablePIDController(NTable table, String name, double _p, double _i, double _d) {
        this.table = table.sub(name);
        if (!this.table.exists("kP", "kI", "kD")) {
            this.table.set("kP", _p);
            this.table.set("kI", _i);
            this.table.set("kD", _d);
            this.table.makePersistent("kP", "kI", "kD");
        }
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
     * specified setpoint and the default tolerance.
     *
     * @param setpoint
     */
    public void setup(double setpoint) {
        setup(setpoint, 0);
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
    public void setup(double setpoint, double tolerance) {
        controller = new PIDController(
                table.getDouble("kP"),
                table.getDouble("kI"),
                table.getDouble("kD"));

        if (continuous) {
            controller.enableContinuousInput(continuousStart, continuousEnd);
        }

        controller.setSetpoint(setpoint);
        table.set("setpoint", setpoint);
        controller.setTolerance(tolerance);
        table.set("tolerance", tolerance);
    }

    /** Resets the PID controller. */
    public void reset() {
        controller.reset();
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

    /** {@return whether this PIDController is within tolerance of the setpoint} */
    public boolean atSetpoint() {
        return this.controller.atSetpoint();
    }

    /** {@return the wrapped PIDController} */
    public PIDController getController() {
        return controller;
    }
}
