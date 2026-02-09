package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
    public static final int TESTING_VOLTS = 1;
    public static final double GEAR_REDUCTION = 1; // initial gear ____T, secondary gear 200T
    public static final PIDController PID = new PIDController(0, 0, 0);
    public static final SimpleMotorFeedforward FF = new SimpleMotorFeedforward(0, 0, 0);
    public static final Angle LOWER_LIMIT = Rotations.of(0);
    public static final Angle UPPER_LIMIT = Rotations.of(1);
}
