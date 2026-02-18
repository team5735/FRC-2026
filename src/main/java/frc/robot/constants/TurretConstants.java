package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class TurretConstants {
    public static final double GEAR_REDUCTION = 200 / 20; // initial gear 20T, secondary gear 200T

    // PID gains (with motion profiling)
    public static final double KP = 15;
    public static final double KI = 0;
    public static final double KD = 0;

    // Simple Feedforward gains (KA is unused but available), all roughly tuned due to SysID misbehavior
    // TODO - retune for full load
    public static final double KS = 0.21;
    public static final double KV = 0.65;
    public static final double KA = 0.15;

    public static final AngularVelocity MAX_VEL = RotationsPerSecond.of(5);
    public static final AngularAcceleration MAX_ACC = RotationsPerSecondPerSecond.of(7.5);
    public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_VEL.in(RotationsPerSecond), MAX_ACC.in(RotationsPerSecondPerSecond));

    // Imaginary hard stops for the turret; all setpoints are to be within a set padding of these
    public static final Angle LOWER_LIMIT = Rotations.of(0);
    public static final Angle UPPER_LIMIT = Rotations.of(1);
    public static final Angle SOFT_PADDING = Rotations.of(0.05);
    
    // Robot-relative starting angle
    public static final Angle START_POS = Rotations.of(0.5);
}
