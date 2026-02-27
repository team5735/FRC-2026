package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * 
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public class Constants {
    public enum DrivetrainType {
        DEVBOT,
        COMPBOT
    }

    // disables drivetrain and autos initialization
    public static boolean BREADBOARD_MODE = false;
    public static DrivetrainType DRIVETRAIN_TYPE = DrivetrainType.COMPBOT;

    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int TURRET_CONTROLLER_PORT = 1;
    public static final int TEST_CONTROLLER_PORT = 2;

    public static final int TURRET_MOTOR_ID = 17;

    public static final double DEADBAND = 0.1;

    public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

    public static final int motor6 = 15;
    public static final int motor27 = 27;

    public static final int LAUNCHER_LEFT_KRAKEN_ID = 1;
    public static final int LAUNCHER_RIGHT_KRAKEN_ID = 2;
}
