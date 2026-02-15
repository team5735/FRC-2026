package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class RobotConstants {
    public static final Translation3d ROBOT_TO_TURRET_CENTER = new Translation3d(
        Inches.of(-7.75), 
        Inches.of(-7.75), 
        Inches.of(16)
        );
}
