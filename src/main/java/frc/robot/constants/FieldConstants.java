package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

// This file provides positions, orientations, distances, and poses to field elements for
// the REBUILT 2026 competition playing field.
// These are based on the AndyMark field specifications

public class FieldConstants {
    private static Distance inch(double inches) {
        return Inches.of(inches);
    }

    /**
     * Transforms a blue-alliance {@link Translation2d} into a red-alliance
     * {@link Translation2d}
     * 
     * @return red-alliance {@link Translation2d}
     */
    public static Translation2d redElement(Translation2d blueElement) {
        return FAR_CORNER.minus(blueElement);
    }

    /**
     * Contains all 32 AprilTags in wpiBlue coordinates.
     *
     * <p>
     * Access the tags with {@link AprilTagFieldLayout#getTags} and
     * {@link AprilTagFieldLayout#getTagPose}
     */
    public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);

    // Dimensions of the arena, got from running FieldMapTest in the tests folder,
    // obtained from the official AprilTagFieldLayout k2026RebuiltAndymark built
    // into wpilib
    // The terms length and width (and height and depth) are ambiguous.
    // Instead of using those, we say the long side "length" of the field is
    // length_x and the short side "width" of the field if length_y.
    // All of our dimensions will be length_x, length_y, and length_z in wpiblue
    // field coordinates meaning downfield is in the x- direction, across the short
    // side of the field is y, and up in the air is z.
    public static final Distance FIELD_LENGTH_X = Meters.of(APRILTAG_FIELD_LAYOUT.getFieldLength());
    public static final Distance FIELD_LENGTH_Y = Meters.of(APRILTAG_FIELD_LAYOUT.getFieldWidth());

    public static final Translation2d FAR_CORNER = new Translation2d(FIELD_LENGTH_X, FIELD_LENGTH_Y);
    public static final Translation2d FIELD_CENTER = FAR_CORNER.div(2);

    // from
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    // NOTE: these are in the wpiBlue coordinate frame
    // use the function redElement() to get the corresponding red alliance element
    // the coordinates for all elements are ALWAYS in the wpiBlue coordinate frame
    // Red "RIGHT" elements are red alliance right ie viewed from the red alliance
    // side
    // So, when viewed from the blue side, the red element will be on the left
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(inch(181.56), inch(158.32));
    public static final Translation2d BLUE_OUTPOST_CENTER = new Translation2d(inch(0), inch(25.62));

    public static final Translation2d BLUE_TRENCH_RIGHT_CENTER = new Translation2d(inch(181.56), inch(24.97));
    public static final Translation2d BLUE_TRENCH_LEFT_CENTER = new Translation2d(inch(181.56),
            inch(FIELD_LENGTH_Y.in(Inches) - 24.97));
    public static final Translation3d TRENCH_DIMENSION = new Translation3d(
            inch(0), // TODO: incorrect
            inch(2 * 24.97), // opening extent along y-axis
            inch(22.25)); // opening extent along z-axis

    public static final Translation2d BLUE_RAMP_RIGHT_CENTER = new Translation2d(inch(181.56),
            inch(24.97 * 2 + 12 + 73 / 2.0));
    public static final Translation2d BLUE_RAMP_LEFT_CENTER = new Translation2d(inch(181.56),
            inch(FIELD_LENGTH_Y.in(Inches) - (24.97 * 2 + 12 + 73 / 2.0)));

    public static final Translation3d RAMP_DIMENSION = new Translation3d(
            inch(44.4), // extent across the x-axis
            inch(73), // extent across the y-axis
            inch(6.513)); // ramp starts at 0 height and goes to this height in the middle
}
