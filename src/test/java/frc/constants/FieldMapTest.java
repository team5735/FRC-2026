package frc.constants;

import org.junit.jupiter.api.Test;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;

public class FieldMapTest {
    public static String pretty(Translation2d d){
        return String.format("(%.2f\", %.2f\")",d.getMeasureX().in(Inches), d.getMeasureY().in(Inches));
    }
    public static String pretty(Pose2d p){
        return String.format("%s %.1f°",pretty(p.getTranslation()), p.getRotation().getDegrees());
    }

    @Test
    void fieldMapTest() throws Exception {
        System.out.printf("Field Center: %s\n",FieldConstants.FIELD_CENTER.toString());
        System.out.printf("\n");

        System.out.printf("Blue hub: %s\n",FieldConstants.BLUE_HUB_CENTER.toString());
        System.out.printf("Red hub: %s\n",FieldConstants.redElement(FieldConstants.BLUE_HUB_CENTER).toString());
        System.out.printf("\n");

        System.out.printf("Blue outpost: %s\n",FieldConstants.BLUE_OUTPOST_CENTER.toString());
        System.out.printf("Red outpost: %s\n",FieldConstants.redElement(FieldConstants.BLUE_OUTPOST_CENTER).toString());
        System.out.printf("\n");

        System.out.printf("Blue trench right: %s\n",FieldConstants.BLUE_TRENCH_RIGHT_CENTER.toString());
        System.out.printf("Red trench right: %s\n",FieldConstants.redElement(FieldConstants.BLUE_TRENCH_RIGHT_CENTER).toString());

        System.out.printf("Blue trench left: %s\n",FieldConstants.BLUE_TRENCH_LEFT_CENTER.toString());
        System.out.printf("Red trench left: %s\n",FieldConstants.redElement(FieldConstants.BLUE_TRENCH_LEFT_CENTER).toString());
        System.out.printf("\n");

        System.out.printf("Blue ramp right: %s\n",FieldConstants.BLUE_RAMP_RIGHT_CENTER.toString());
        System.out.printf("Red ramp right: %s\n",FieldConstants.redElement(FieldConstants.BLUE_RAMP_RIGHT_CENTER).toString());

        System.out.printf("Blue ramp left: %s\n",FieldConstants.BLUE_RAMP_LEFT_CENTER.toString());
        System.out.printf("Red ramp left: %s\n",FieldConstants.redElement(FieldConstants.BLUE_RAMP_LEFT_CENTER).toString());

        AprilTagFieldLayout atf = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        System.out.printf("april tags:\n");
        for (var a:atf.getTags()){
            System.out.printf("  %s\n",a.toString());
        }
        System.out.printf("Field length: %f\n",atf.getFieldLength());
        System.out.printf("Field width: %f\n",atf.getFieldWidth());

        System.out.printf("Blue Tower Front Center: %s\n", pretty(FieldConstants.BLUE_TOWER_FRONT_CENTER));
        System.out.printf("  Climber Hook Min Z: %.2f\"\n", FieldConstants.CLIMBER_HOOK_MIN_Z.in(Inches));
        System.out.printf("  Climber Localization Pose: %s\n", pretty(FieldConstants.CLIMBER_BLUE_LOCALIZATION_POSE));
        System.out.printf("  Climber Left Align Pose: %s\n", pretty(FieldConstants.CLIMBER_BLUE_LEFT_CLIMB_ALIGN_POSE));
        System.out.printf("  Climber Left Climb Pose: %s\n", pretty(FieldConstants.CLIMBER_BLUE_LEFT_CLIMB_POSE));

        System.out.printf("Red Tower Front Center: %s\n", pretty(FieldConstants.redElement(FieldConstants.BLUE_TOWER_FRONT_CENTER)));
        System.out.printf("  Climber Hook Min Z: %.2f\"\n", FieldConstants.CLIMBER_HOOK_MIN_Z.in(Inches));
        System.out.printf("  Climber Localization Pose: %s\n", pretty(FieldConstants.redElement(FieldConstants.CLIMBER_BLUE_LOCALIZATION_POSE)));
        System.out.printf("  Climber Left Align Pose: %s\n", pretty(FieldConstants.redElement(FieldConstants.CLIMBER_BLUE_LEFT_CLIMB_ALIGN_POSE)));
        System.out.printf("  Climber Left Climb Pose: %s\n", pretty(FieldConstants.redElement(FieldConstants.CLIMBER_BLUE_LEFT_CLIMB_POSE)));

    }
}
