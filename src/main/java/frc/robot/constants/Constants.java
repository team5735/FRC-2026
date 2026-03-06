package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    // determines which TimedRobot to use
    public static enum BotConfiguration {
        FULL_DEVBOT,
        FULL_COMPBOT,
        HOOD,
        HOOD_PEEK_A_BOO,
    }

    public static final BotConfiguration CURRENT_ROBOT = BotConfiguration.HOOD_PEEK_A_BOO;

    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int SUBSYSTEM_CONTROLLER_PORT = 1;
    public static final int TEST_CONTROLLER_PORT = 2;

    // The range of servo setpoints that correspond to hood lowest
    // and highest positions. The servo can physically be set
    // between 0 and 1. But these limit that by the physical reality
    // of the hood
    public static final double HOOD_LOWEST_SERVO_POSITION = 0.6;
    public static final double HOOD_HIGHEST_SERVO_POSITION = 1.0;

    // The range of angles (in degrees) that the physcial shooter hood can
    // move to. While tuning the hood servo, these don't change
    public static final double HOOD_LOWEST_ANGLE_DEGREES = 8.0;
    public static final double HOOD_HIGHEST_ANGLE_DEGREES = 45.0;

    public static final int INTAKE_SLAPDOWN_TALONFX_ID = 58;
    public static final int INTAKE_ROLLER_TALONFX_ID = 21;

    public static final double START_REVOLUTION_POSITION = 0.4;
    public static final double END_REVOLUTION_POSITION = 0.6;

    public static final double DEADBAND = 0.1;

    public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

    public static final int motor6 = 15;
    public static final int motor27 = 27;
    public static final int SPINDEX_FEED_VORTEX_ID = 30;
    public static final int SPINDEX_WHEEL_VORTEX_ID = 31;

    public static final int LAUNCHER_LEFT_KRAKEN_ID = 21;
    public static final int LAUNCHER_RIGHT_KRAKEN_ID = 20;

    public static final int TURRET_MOTOR_ID = 22;
    public static final int TURRET_LIMIT_PIN = 2;

    public static final int CLIMB_TALON_ID = 24;
    public static final int CLIMB_UPPER_LIMIT_PIN = 0;
    public static final int CLIMB_LOWER_LIMIT_PIN = 1;
}
