package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public enum DrivetrainType {
        DEVBOT,
        COMPBOT
    }

    public static DrivetrainType DRIVETRAIN_TYPE = DrivetrainType.DEVBOT;

    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int SUBSYSTEM_CONTROLLER_PORT = 1;
    public static final int TEST_CONTROLLER_PORT = 2;

    public static final double DEADBAND = 0.1;

    public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

    // if you don't feel like picking for PIDs
    public static final double TOLERANCE = 0.005;

    public static final double PID_P = 6;
    public static final double PID_I = 2;
    public static final double PID_D = 0;

    public static final double PROFILED_PID_P = 2;
    public static final double PROFILED_PID_I = 0;
    public static final double PROFILED_PID_D = 0;

    public static final double MAX_VELOCITY = 1;
    public static final double MAX_ACCELERATION = 1;
}
