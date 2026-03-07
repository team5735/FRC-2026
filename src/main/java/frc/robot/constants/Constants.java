package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    // determines which TimedRobot to use
    public static enum BotConfiguration {
        FULL_DEVBOT,
        FULL_COMPBOT,
        HOOD,
        HOOD_PEEK_A_BOO,
        INTAKE,
    }

    public static final BotConfiguration CURRENT_ROBOT = BotConfiguration.HOOD_PEEK_A_BOO;

    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int SUBSYSTEM_CONTROLLER_PORT = 1;
    public static final int TEST_CONTROLLER_PORT = 2;

    public static final int INTAKE_SLAPDOWN_TALONFX_ID = 58;
    public static final int INTAKE_ROLLER_TALONFX_ID = 21;

    public static final double START_REVOLUTION_POSITION = 0.4;
    public static final double END_REVOLUTION_POSITION = 0.6;

    public static final double DEADBAND = 0.1;

    public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

    public static final int SPINDEX_FEED_VORTEX_ID = 30;
    public static final int SPINDEX_WHEEL_VORTEX_ID = 31;

    public static final int LAUNCHER_LEFT_KRAKEN_ID = 21;
    public static final int LAUNCHER_RIGHT_KRAKEN_ID = 20;

    public static final int TURRET_MOTOR_ID = 22;
    public static final int TURRET_LIMIT_PIN = 2;

    public static final int CLIMB_TALON_ID = 24;
    public static final int CLIMB_UPPER_LIMIT_PIN = 0;
    public static final int CLIMB_LOWER_LIMIT_PIN = 1;

    public static final int HOOD_SERVO_PIN = 0;
    public static final int HOOD_FEEDBACK_PIN = 0;
}
