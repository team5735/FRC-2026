package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class TunableProfiledPIDController {
    /** The wrapped PID controller. */
    protected ProfiledPIDController controller;

    /** The table used to manage PID values. */
    private NTable table;

    /** Creates a new profiled PID controller with the zeroed defaults in /pid/{name}. */
    public TunableProfiledPIDController(String name) {
        this(NTable.root("pid"), name, 0, 0, 0, 0, 0);
    }

    /** Creates a new profiled PID controller with the specified defaults in /pid/{name}. */
    public TunableProfiledPIDController(String name, double p, double i, double d) {
        this(NTable.root("pid"), name, p, i, d, 0, 0);
    }

    /** Creates a new profiled PID controller with the specified defaults in /pid/{name}. */
    public TunableProfiledPIDController(String name, double vel, double accel) {
        this(NTable.root("pid"), name, 0, 0, 0, vel, accel);
    }

    /** Creates a new profiled PID controller with the specified defaults in /pid/{name}. */
    public TunableProfiledPIDController(String name, double p, double i, double d, double vel, double accel) {
        this(NTable.root("pid"), name, p, i, d, vel, accel);
    }

    /** Creates a new profiled PID controller with the zeroed defaults in {table}/{name}. */
    public TunableProfiledPIDController(NTable table, String name) {
        this(table, name, 0, 0, 0, 0, 0);
    }

    /** Creates a new profiled PID controller with the specified values in {table}/{name}. */
    public TunableProfiledPIDController(NTable table, String name, double p, double i, double d) {
        this(table, name, p, i, d, 0, 0);
    }

    /** Creates a new profiled PID controller with the specified values in {table}/{name}. */
    public TunableProfiledPIDController(NTable table, String name, double vel, double accel) {
        this(table, name, 0, 0, 0, vel, accel);
    }

    /** Creates a new profiled PID controller with the specified values in {table}/{name}. */
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
     * @param goalPos
     */
    public void setup(double goalPos) {
        setup(new State(goalPos, 0), 0);
    }

    /**
     * Set up this PID controller.
     *
     * <p>
     * This retrieves the PID constants from the table created during construction.
     * It also telemeterizes the setpoint and sets the controller to have the
     * specified setpoint and tolerance.
     *
     * @param goal
     * @param tolerance
     */
    public void setup(State goal, double tolerance) {
        controller = new ProfiledPIDController(
                table.getDouble("kP"),
                table.getDouble("kI"),
                table.getDouble("kD"),
                new TrapezoidProfile.Constraints(
                        table.getDouble("max velocity"),
                        table.getDouble("max acceleration")));

        controller.setGoal(goal);
        table.set("goal position", goal.position);
        table.set("goal velocity", goal.velocity);
        controller.setTolerance(tolerance);
    }

    /**
     * Sets the goal of the actively parameterized profiled PID controller.
     * 
     * @param goal
     */
    public void setGoal(State goal){
        controller.setGoal(goal);
        table.set("goal position", goal.position);
        table.set("goal velocity", goal.velocity);
    }

    /**
     * Sets the tolerance of the actively parameterized profiled PID controller.
     * 
     * @param tolerance
     */
    public void setTolerance(double tolerance){
        controller.setTolerance(tolerance);
    }

    /** Resets the PID controller. */
    public void reset(double position) {
        controller.reset(position, 0);
    }

    /** Resets the PID controller. */
    public void reset(double position, double velocity) {
        controller.reset(position, velocity);
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
     * @param position measured position of the system
     * @param goal goal state of the system
     * @return The controller output, or zero if atSetpoint.
     */
    public double calculate(double position, State goal) {
        double output = controller.calculate(position, goal);
        table.set("position", position);
        table.set("goal position", goal.position);
        table.set("goal velocity", goal.velocity);
        table.set("output", output);
        return output;
    }

    /**
     * Returns the next output of the PID controller.
     * 
     * <p>
     * Also telemeterizes the given and PID controller output.
     * 
     * @param position measured position of the system
     * @return The controller output, or zero if atSetpoint.
     */
    public double calculate(double position){
        double output = controller.calculate(position);
        table.set("position", position);
        table.set("output", output);
        return output;
    }

    /** {@return whether this ProfiledPIDController is within tolerance of the setpoint} */
    public boolean atSetpoint() {
        return controller.atSetpoint();
    }
    /**
     * @return whether this ProfiledPIDController is within tolerance of its goal and has finished its trajectory
     */
    public boolean atGoal(){
        return controller.atGoal();
    }

    /** {@return the wrapped ProfiledPIDController} */
    public ProfiledPIDController getController() {
        return controller;
    }
}
