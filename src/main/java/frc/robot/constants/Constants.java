package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public enum DrivetrainType {
        DEVBOT,
        COMPBOT
    }

    // disables drivetrain and autos initialization
    public static boolean BREADBOARD_MODE = false;
    public static DrivetrainType DRIVETRAIN_TYPE = DrivetrainType.DEVBOT;

    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int SUBSYSTEM_CONTROLLER_PORT = 1;
    public static final int TEST_CONTROLLER_PORT = 2;

    public static final int INTAKE_TALONFX_ID = 58;
    public static final int INTAKE_ROLLER_TALONFX_ID = 21;

    public static final double DEADBAND = 0.1;

    public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

    public static final int motor6 = 15;
    public static final int motor27 = 27;
}
