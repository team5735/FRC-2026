package frc.robot.constants;

public class HoodConstants {
    // (trying to make these the new way of tuning the hood)
    // this is the range of servo setpoints that bring the hood from 
    // positions 0 to 1. Meaning, the hood's lowest and highest positions, 
    // normalized to the range [0,1]
    // set this a little higher to expand the range of hood positions
    // set a little lower to limit the range
    public static final double SERVO_SETPOINT_TO_HOOD_POS_RATIO = 0.526; // 0.75-0.224;
    public static final double SERVO_SETPOINT_AT_HOOD_0 = 0.224;

    // (trying to deprecate these)
    // The range of servo setpoints that correspond to hood lowest
    // and highest positions. The servo can physically be set
    // between 0 and 1. But these limit that by the physical reality
    // of the hood
    public static final double LOWEST_SERVO_POSITION = SERVO_SETPOINT_AT_HOOD_0;
    public static final double HIGHEST_SERVO_POSITION = SERVO_SETPOINT_AT_HOOD_0 + SERVO_SETPOINT_TO_HOOD_POS_RATIO;

    // The range of angles (in degrees) that the physcial shooter hood can
    // move to. While tuning the hood servo, these don't change
    public static final double LOWEST_ANGLE_DEGREES = 8.0;
    public static final double HIGHEST_ANGLE_DEGREES = 45.0;

    // TODO: test
    public static final double ANGLE_AT_ARC = 20.0;

    // 0.45 = 1.75v
    // do this twice and you get a linear interpolation between feedback
    // voltage and servo setpoint
    public static final double SERVO_VOLTAGE_REF0 = 0.45; // setpoint
    public static final double SERVO_VOLTAGE_AT_REF0 = 1.75; // todo
    public static final double SERVO_VOLTAGE_REF1 = 0.775; // setpoint
    public static final double SERVO_VOLTAGE_AT_REF1 = 1.00; // todo

    public static final double DYNAMIC_TOLERANCE_DEGREES = 5;
}
