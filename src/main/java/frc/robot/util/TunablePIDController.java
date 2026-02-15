package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.Constants;

public class TunablePIDController {
    /** The wrapped PID controller. */
    protected PIDController controller;

    /** The table used to manage PID values. */
    private NTable table;

    /** Creates a new PID controller with the default values in /pid/{name}. */
    public TunablePIDController(String name) {
        this(NTable.root("pid"), name, Constants.PID_P, Constants.PID_I, Constants.PID_D);
    }

    /** Creates a new PID controller with the specified values in /pid/{name}. */
    public TunablePIDController(String name, double _p, double _i, double _d) {
        this(NTable.root("pid"), name, _p, _i, _d);
    }

    /** Creates a new PID controller with the default values in {table}/{name}. */
    public TunablePIDController(NTable table, String name) {
        this(table, name, Constants.PID_P, Constants.PID_I, Constants.PID_D);
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
        setup(setpoint, Constants.TOLERANCE);
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
