package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
    public static final double ROLL_IN_VOLTS = 8.0;
    public static final double ROLL_OUT_VOLTS = -6.0;

    public static final double KS = 0; //TODO
    public static final double KV = 0; //TODO
    public static final double KG = 0; //TODO

    public static final Angle START_POS = Degrees.of(195);
    public static final Angle UPPER_RELEASE_POS = Degrees.of(180);
    public static final Angle LOWER_RELEASE_POS = Degrees.of(105);

    public static final Angle ANGULAR_TOLERANCE = Degrees.of(5);

    public static final AngularVelocity LIFT_VEL = RotationsPerSecond.of(0.25);
    public static final AngularVelocity SLAPDOWN_VEL = RotationsPerSecond.of(0.25);

    public static final double GEAR_REDUCTION = 61.25; //TODO = verify
}
