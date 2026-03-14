package frc.robot.constants;

public class HoodConstants {
    // The range of servo setpoints that correspond to hood lowest
    // and highest positions. The servo can physically be set
    // between 0 and 1. But these limit that by the physical reality
    // of the hood
    public static final double LOWEST_SERVO_POSITION = 0.12555;
    public static final double HIGHEST_SERVO_POSITION = 0.65;

    // The range of angles (in degrees) that the physcial shooter hood can
    // move to. While tuning the hood servo, these don't change
    public static final double LOWEST_ANGLE_DEGREES = 8.0;
    public static final double HIGHEST_ANGLE_DEGREES = 45.0;

    // TODO: test
    public static final double ANGLE_AT_ARC = 8.0;
    public static final double POS_AT_ARC = 0.575;

    // 0.4 = 1.8v
    public static final double SERVO_VOLTAGE_REF0 = 0.45; // setpoint
    public static final double SERVO_VOLTAGE_AT_REF0 = 1.75; // todo
    public static final double SERVO_VOLTAGE_REF1 = 0.775; // setpoint
    public static final double SERVO_VOLTAGE_AT_REF1 = 1.00; // todo

    public static final double DYNAMIC_TOLERANCE_DEGREES = 5;
}
