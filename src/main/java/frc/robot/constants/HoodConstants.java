package frc.robot.constants;

public class HoodConstants {
    // The range of servo setpoints that correspond to hood lowest
    // and highest positions. The servo can physically be set
    // between 0 and 1. But these limit that by the physical reality
    // of the hood
    public static final double LOWEST_SERVO_POSITION = 0.4;
    public static final double HIGHEST_SERVO_POSITION = 0.6;

    // The range of angles (in degrees) that the physcial shooter hood can
    // move to. While tuning the hood servo, these don't change
    public static final double LOWEST_ANGLE_DEGREES = 8.0;
    public static final double HIGHEST_ANGLE_DEGREES = 45.0;

    // TODO: test
    public static final double ANGLE_AT_ARC = 8.0;
    public static final double POS_AT_ARC = 0.575;
}
