package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TunableProfiledPIDController {
    /** The wrapped PID controller. */
    protected ProfiledPIDController controller;

    /** The table used to manage PID values. */
    private NTable table;

    /** Creates a new PID controller with the zeroed defaults in /pid/{name}. */
    public TunableProfiledPIDController(String name) {
        this(NTable.root("pid"), name, 0, 0, 0, 0, 0);
    }

    /** Creates a new PID controller with the specified defaults in /pid/{name}. */
    public TunableProfiledPIDController(String name, double p, double i, double d) {
        this(NTable.root("pid"), name, p, i, d, 0, 0);
    }

    /** Creates a new PID controller with the specified defaults in /pid/{name}. */
    public TunableProfiledPIDController(String name, double vel, double accel) {
        this(NTable.root("pid"), name, 0, 0, 0, vel, accel);
    }

    /** Creates a new PID controller with the specified defaults in /pid/{name}. */
    public TunableProfiledPIDController(String name, double p, double i, double d, double vel, double accel) {
        this(NTable.root("pid"), name, p, i, d, vel, accel);
    }

    /** Creates a new PID controller with the zeroed defaults in {table}/{name}. */
    public TunableProfiledPIDController(NTable table, String name) {
        this(table, name, 0, 0, 0, 0, 0);
    }

    /** Creates a new PID controller with the specified values in {table}/{name}. */
    public TunableProfiledPIDController(NTable table, String name, double p, double i, double d) {
        this(table, name, p, i, d, 0, 0);
    }

    /** Creates a new PID controller with the specified values in {table}/{name}. */
    public TunableProfiledPIDController(NTable table, String name, double vel, double accel) {
        this(table, name, 0, 0, 0, vel, accel);
    }

    /** Creates a new PID controller with the specified values in {table}/{name}. */
    public TunableProfiledPIDController(NTable table, String name, double p, double i, double d,
            double maxVelocity, double maxAcceleration) {
        this.table = table.sub(name);
        if (!this.table.exists("kP", "kI", "kD", "max velocity", "max acceleration")) {
            this.table.set("kP", p);
            this.table.set("kI", i);
            this.table.set("kD", d);

            this.table.set("max velocity", maxVelocity);
            this.table.set("max acceleration", maxAcceleration);
            this.table.makePersistent("kP", "kI", "kD", "max velocity", "max acceleration");
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
        controller = new ProfiledPIDController(
                table.getDouble("kP"),
                table.getDouble("kI"),
                table.getDouble("kD"),
                new TrapezoidProfile.Constraints(
                        table.getDouble("max velocity"),
                        table.getDouble("max acceleration")));

        controller.setGoal(setpoint);
        table.set("setpoint", setpoint);
        controller.setTolerance(tolerance);
    }

    /** Resets the PID controller. */
    public void reset(double measurement) {
        controller.reset(measurement, 0);
    }

    /** Resets the PID controller. */
    public void reset(double measurement, double velocity) {
        controller.reset(new TrapezoidProfile.State(measurement, velocity));
    }

    /** Resets the PID controller. */
    public void reset(TrapezoidProfile.State state) {
        controller.reset(state);
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
        return value;
    }

    /** {@return whether this ProfiledPIDController is within tolerance of the setpoint} */
    public boolean atSetpoint() {
        return this.controller.atSetpoint();
    }

    /** {@return the wrapped ProfiledPIDController} */
    public ProfiledPIDController getController() {
        return controller;
    }
}
