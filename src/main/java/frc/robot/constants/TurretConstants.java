package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class TurretConstants {
    public static final double GEAR_REDUCTION = 200 / 20; // initial gear 20T, secondary gear 200T

    // PID gains (with motion profiling)
    public static final double KP = 10;
    public static final double KI = 0;
    public static final double KD = 0.65;

    // Simple Feedforward gains, all roughly tuned due to SysID misbehavior
    public static final double KS = 0.21;
    public static final double KV = 0.856;
    public static final double KA = 0.12;

    public static final AngularVelocity MAX_VEL = RotationsPerSecond.of(1);
    public static final AngularAcceleration MAX_ACC = RotationsPerSecondPerSecond.of(3.5);
    public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_VEL.in(RotationsPerSecond), MAX_ACC.in(RotationsPerSecondPerSecond));

    // Imaginary, robot-relative hard stops for the turret; all setpoints are to be
    // within a set padding of these
    public static final Angle REVERSE_LIMIT_BOT_REL = Rotations.of(0.48); // limit the turret would hit whild driving CW
    public static final Angle FORWARD_LIMIT_BOT_REL = Rotations.of(0.306); // limit the turret would hit while driving
                                                                           // CCW
    public static final Angle ZERO_OFFSET = Rotations
            .of((REVERSE_LIMIT_BOT_REL.in(Rotations) + FORWARD_LIMIT_BOT_REL.in(Rotations)) / 2);
    public static final Angle REVERSE_LIMIT_TUR_REL = REVERSE_LIMIT_BOT_REL.minus(ZERO_OFFSET);
    public static final Angle FORWARD_LIMIT_TUR_REL = Rotations.of(1).minus(REVERSE_LIMIT_TUR_REL);
    public static final Angle SOFT_PADDING = Rotations.of(0.025);

    // Ideal robot-relative starting angle
    public static final Angle START_POS_BOT_REL = Rotations.of(0.75);

    /**
     * Clamps a goal angle and to the functional range of this subsystem, converting
     * it to the turret-relative space
     *
     * @param input the robot-relative goal position to be clamped.
     *
     * @return turret-relative {@link Angle} inclusively between the upper and lower
     *         bounds of this
     *         subsystem; either the rotational equivalent of the input, or one of
     *         the bounds.
     */
    public static Angle formatInputRobotRel(Angle input) {
        return formatInputTurretRel(input.minus(ZERO_OFFSET));
    }

    /**
     * Clamps a goal angle and to the functional range of this subsystem
     *
     * @param input the turret-relative goal position to be clamped.
     *
     * @return turret-relative {@link Angle} inclusively between the upper and lower
     *         bounds of this
     *         subsystem; either the rotational equivalent of the input, or one of
     *         the bounds.
     */
    public static Angle formatInputTurretRel(Angle input) {
        double softLowerLimit = MathUtil.inputModulus(REVERSE_LIMIT_TUR_REL.plus(SOFT_PADDING).in(Rotations), 0, 1);
        double softUpperLimit = MathUtil.inputModulus(FORWARD_LIMIT_TUR_REL.minus(SOFT_PADDING).in(Rotations), 0, 1);
        double moddedInput = MathUtil.inputModulus(input.in(Rotations), 0, 1);
        return Rotations.of(MathUtil.clamp(moddedInput, softLowerLimit, softUpperLimit));
    }

    /**
     * Converts a turret-relative {@link Angle} to a robot-relative one
     *
     * @param input the turret-relative position to be converted
     * @return a robot-relative equivalent of the input, calculated by adding
     *         {@link #ZERO_OFFSET}
     */
    public static Angle turretRelToRobotRel(Angle input) {
        return Rotations.of(MathUtil.inputModulus(input.plus(ZERO_OFFSET).in(Rotations), 0, 1));
    }

    /**
     * Converts a robot-relative {@link Angle} to a turret-relative one
     *
     * @param input the robot-relative position to be converted
     * @return a turret-relative equivalent of the input, calculated by subtracting
     *         {@link #ZERO_OFFSET}
     */
    public static Angle robotRelToTurretRel(Angle input) {
        return Rotations.of(MathUtil.inputModulus(input.minus(ZERO_OFFSET).in(Rotations), 0, 1));
    }
}
