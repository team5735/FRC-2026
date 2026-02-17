package frc.robot.constants;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

public class TurretConstants {
    public static final double GEAR_REDUCTION = 200 / 20; // initial gear 20T, secondary gear 200T
    public static final double KP = 15;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0.21;
    public static final double KV = 0.65;
    public static final double KA = 0.15;
    public static final AngularVelocity MAX_VEL = RotationsPerSecond.of(1);
    public static final AngularAcceleration MAX_ACC = RotationsPerSecondPerSecond.of(1.5);
    public static final Angle LOWER_LIMIT = Rotations.of(0);
    public static final Angle UPPER_LIMIT = Rotations.of(0.75);
    // VERY rough estimate based on ABS density and disc approximation for the gear
    // TODO - Recalc for real turret
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(1.490e-4);
    public static final Angle SOFT_PADDING = Rotations.of(0.05);
    public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_VEL.in(RotationsPerSecond), MAX_ACC.in(RotationsPerSecondPerSecond));
    public static final Angle START_POS = Rotations.of(0.5);
}
