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

    public static final int INTAKE_TALONFX_ID = 58;
    public static final int INTAKE_SPARKMAX_ID = 21;

    public static final double DEADBAND = 0.1;

    public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

    // these numbers are 0 so you notice when you haven't paid attention to them
    public static final double TOLERANCE = 0;

    public static final double PID_P = 0;
    public static final double PID_I = 0;
    public static final double PID_D = 0;

    public static final double PROFILED_PID_P = 0;
    public static final double PROFILED_PID_I = 0;
    public static final double PROFILED_PID_D = 0;

    public static final double MAX_VELOCITY = 0;
    public static final double MAX_ACCELERATION = 0;
}
