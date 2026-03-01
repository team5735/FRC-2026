package frc.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.FieldConstants;

public class FieldMapTest {
    public static String pretty(Translation2d d) {
        return String.format("(%.2f\", %.2f\")", d.getMeasureX().in(Inches), d.getMeasureY().in(Inches));
    }

    public static String pretty(Pose2d p) {
        return String.format("%s %.1f°", pretty(p.getTranslation()), p.getRotation().getDegrees());
    }

    private static String pretty(Translation3d d) {
        return String.format("(%.2f\", %.2f\", %.2f\")", d.getMeasureX().in(Inches), d.getMeasureY().in(Inches),
                d.getMeasureZ().in(Inches));
    }

    private static String pretty(Rotation3d r) {
        return String.format("(%.2f°, %.2f°, %.2f°)",
                r.getMeasureX().in(Degrees), r.getMeasureY().in(Degrees), r.getMeasureZ().in(Degrees));
    }

    private static String pretty(Pose3d p) {
        return String.format("%s %s", pretty(p.getTranslation()), pretty(p.getRotation()));
    }

    private static Translation2d red(Translation2d in) {
        return FieldConstants.redElement(in);
    }

    private static Pose2d red(Pose2d in) {
        return FieldConstants.redElement(in);
    }

    private static Rectangle2d red(Rectangle2d in) {
        return FieldConstants.redElement(in);
    }

    @Test
    void fieldMapTest() throws Exception {
        System.out.println("Field Center: " + FieldConstants.FIELD_CENTER);
        System.out.println();

        System.out.println("Blue hub: " + FieldConstants.BLUE_HUB_CENTER);
        System.out.println("Red hub: " + red(FieldConstants.BLUE_HUB_CENTER));
        System.out.println();

        System.out.println("Blue outpost: " + FieldConstants.BLUE_OUTPOST_CENTER);
        System.out.println("Red outpost: " + red(FieldConstants.BLUE_OUTPOST_CENTER));
        System.out.println();

        System.out.println("Blue trench right: " + FieldConstants.BLUE_TRENCH_RIGHT_CENTER);
        System.out.println("Red trench right: " + red(FieldConstants.BLUE_TRENCH_RIGHT_CENTER));

        System.out.println("Blue trench left: " + FieldConstants.BLUE_TRENCH_LEFT_CENTER);
        System.out.println("Red trench left: " + red(FieldConstants.BLUE_TRENCH_LEFT_CENTER));
        System.out.println();

        System.out.printf("Blue trench right exclusion zone: %s\n",
                FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_RIGHT.toString());
        System.out.printf("Red trench right exclusion zone: %s\n",
                red(FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_RIGHT).toString());
        System.out.println();

        System.out.println("Blue ramp right: " + FieldConstants.BLUE_RAMP_RIGHT_CENTER);
        System.out.println("Red ramp right: " + red(FieldConstants.BLUE_RAMP_RIGHT_CENTER));

        System.out.println("Blue ramp left: " + FieldConstants.BLUE_RAMP_LEFT_CENTER);
        System.out.println("Red ramp left: " + red(FieldConstants.BLUE_RAMP_LEFT_CENTER));

        AprilTagFieldLayout atf = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        System.out.println("AprilTags:");
        for (AprilTag tag : atf.getTags()) {
            System.out.printf("  " + tag.ID + " = " + pretty(tag.pose));
        }

        System.out.println("Field length: " + atf.getFieldLength());
        System.out.println("Field width: " + atf.getFieldWidth());

        System.out.println("Blue-alliance climber poses:");
        System.out.println("  Blue tower front center: " + pretty(FieldConstants.BLUE_TOWER_FRONT_CENTER));
        System.out.println("  Hook min Z: " + FieldConstants.CLIMBER_HOOK_MIN_Z.in(Inches) + "\"");
        System.out.println("  Localization pose: " + pretty(FieldConstants.CLIMBER_BLUE_LOCALIZATION_POSE));
        System.out.println("  Left align pose: " + pretty(FieldConstants.CLIMBER_BLUE_LEFT_CLIMB_ALIGN_POSE));
        System.out.println("  Left climb pose: " + pretty(FieldConstants.CLIMBER_BLUE_LEFT_CLIMB_POSE));

        System.out.println("Red-alliance climber poses:");
        System.out.println("  Red Tower Front Center: " + pretty(red(FieldConstants.BLUE_TOWER_FRONT_CENTER)));
        System.out.println("  Hook min Z: " + FieldConstants.CLIMBER_HOOK_MIN_Z.in(Inches) + "\"");
        System.out.println("  Localization pose: " + pretty(red(FieldConstants.CLIMBER_BLUE_LOCALIZATION_POSE)));
        System.out.println("  Left align pose: " + pretty(red(FieldConstants.CLIMBER_BLUE_LEFT_CLIMB_ALIGN_POSE)));
        System.out.println("  Left climb pose: " + pretty(red(FieldConstants.CLIMBER_BLUE_LEFT_CLIMB_POSE)));
    }

    @Test
    void withinRectangleTest() throws Exception {
        Rectangle2d r = new Rectangle2d(new Pose2d(0, 0, Rotation2d.kZero), 10, 20);
        Translation2d p = new Translation2d(0, 0);

        Translation2d ll = r.getCenter().getTranslation().minus(new Translation2d(r.getXWidth()/2, r.getYWidth()/2));
        Translation2d ur = r.getCenter().getTranslation().plus(new Translation2d(r.getXWidth()/2, r.getYWidth()/2));
        System.out.printf("Rectangle: (%s)x(%s)\n", ll, ur);

        for (int dy = -10; dy <= 10; dy += 5) {
            for (int dx = -10; dx <= 10; dx += 5) {
                var pp = p.plus(new Translation2d(dx, dy));
                boolean in = r.contains(pp);
                System.out.printf("(%.1f %.1f): in? %s\n", pp.getX(), pp.getY(), in);
            }
        }
    }
}
