package frc.constants;

import java.nio.file.Path;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Meters;

import frc.robot.util.fieldmap.FieldFmapReader;
import frc.robot.util.fieldmap.FieldAprilTag;
import frc.robot.util.fieldmap.FieldSpec;
import frc.robot.constants.FieldConstants;

public class FieldMapTest {
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

    }
}
