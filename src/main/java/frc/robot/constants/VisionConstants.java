package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class VisionConstants {
    public static final String  LIMELIGHTS[] = { "limelight-left" };
    public static final boolean IS_MT2       = false;

    public static final Distance        TOLERATED_HEIGHT          = Centimeters.of(25);
    public static final AngularVelocity TOLERATED_ROTATIONAL_RATE = DegreesPerSecond.of(10);
    public static final AngularVelocity MAX_ROTATION_SPEED        = DegreesPerSecond.of(1);

    public static final Time            MIN_RESET_DELAY          = Seconds.of(5);
    public static final Distance        NEAR_ENOUGH_TO_RESET     = Meters.of(1.5);
    public static final Angle           MAX_YAW_STDDEV_FOR_RESET = Degrees.of(5);
    public static final AngularVelocity MAX_FOR_RESET            = DegreesPerSecond.of(0.5);

    public static final double SINGLE_TAG_MAX_AMBIGUITY = 0.2;
    public static final double MULTI_TAG_MAX_AMBIGUITY  = 0.5;
    public static final double MAX_AMBIGUITY_FOR_RESET  = 0.2;
}
