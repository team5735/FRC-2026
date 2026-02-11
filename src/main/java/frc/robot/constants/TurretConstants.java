package frc.robot.constants;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import yams.math.ExponentialProfilePIDController;

public class TurretConstants {
    public static final int TESTING_VOLTS = 1;
    public static final double GEAR_REDUCTION = 200/20; // initial gear 20T, secondary gear 200T
    public static final ExponentialProfilePIDController EXPO_PID = new ExponentialProfilePIDController(
            0,
            0,
            0,
            ExponentialProfilePIDController.createConstraints(
                    Volts.of(10),
                    RotationsPerSecond.of(0),
                    RotationsPerSecondPerSecond.of(0)));
    public static final ExponentialProfilePIDController SIM_EXPO_PID = new ExponentialProfilePIDController(
            0,
            0,
            0,
            ExponentialProfilePIDController.createConstraints(
                    Volts.of(10),
                    RotationsPerSecond.of(0.25),
                    RotationsPerSecondPerSecond.of(0.5)));

    public static final SimpleMotorFeedforward FF = new SimpleMotorFeedforward(0, 0, 0);
    public static final Angle LOWER_LIMIT = Rotations.of(0);
    public static final Angle UPPER_LIMIT = Rotations.of(1);
    // VERY rough estimate based on ABS density and disc approximation for the gear
    // TODO - Recalc for real turret
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(1.490e-4); 
}
