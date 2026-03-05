package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;

public class RobotConstants {
    public static final Translation2d ROBOT_TO_TURRET_CENTER = switch (Constants.DRIVETRAIN_TYPE) {
        case DEVBOT -> new Translation2d(
                Inches.of(-7.75),
                Inches.of(-7.75));
        case COMPBOT -> new Translation2d(
                Inches.of(-4.245),
                Inches.of(6.5));
    };
}
