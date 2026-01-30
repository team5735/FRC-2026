package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

// This file provides positions, orientations, distances, and poses to field elements for
// the REBUILT 2026 competition playing field.
// These are based on the AndyMark field specifications

// all coordinates are in the wpiBlue coordinate frame, with (0,0,0) being blue alliance right corner
public class FieldConstants {
    private static double in2m(double inches) {return inches*2.54/100.0;}

    // gets the red alliance equivalent of the given blue field piece
    // the returned dimensions are STILL in wpiBlue coords
    public static Translation2d redElement(Translation2d blueElement){
        return new Translation2d(FIELD_LENGTH_X.in(Meters) - blueElement.getX(),
                                 FIELD_LENGTH_Y.in(Meters) - blueElement.getY());
    }


    // dimensions of the arena, got from running FieldMapTest from the official AprilTagFieldLayout k2026RebuiltAndymark
    // length and width (and height and depth) are ambiguous. Instead the long side "length" of the field is length_x
    // and the short side "width" of the field if length_y
    // all of our dimensions will be length_x, length_y, and  length_z in wpiblue field coordinates
    public static final Distance FIELD_LENGTH_X = Meters.of(16.518); // extent along the x-axis
    public static final Distance FIELD_LENGTH_Y = Meters.of(8.043);   // extent along the y-axis

    public static final Translation2d FIELD_CENTER = new Translation2d(FIELD_LENGTH_X.div(2), FIELD_LENGTH_Y.div(2));

    // from https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    // NOTE: we are only specifying blue elements.
    //       use the function redElement() to get the corresponding red alliance element.
    //       the coordinates for all elements are ALWAYS in the wpiBlue coordinate frame
    //       Red "RIGHT" elements are red alliance right ie viewed from the red alliance side
    //       So, when viewed from the blue side, the red element will be on the left
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(in2m(181.56), in2m(158.32));
    public static final Translation2d BLUE_OUTPOST_CENTER = new Translation2d(in2m(0), in2m(25.62));

    public static final Translation2d BLUE_TRENCH_RIGHT_CENTER = new Translation2d(in2m(181.56), in2m(24.97));
    public static final Translation2d BLUE_TRENCH_LEFT_CENTER = new Translation2d(in2m(181.56), in2m(FIELD_LENGTH_Y.in(Inches)-24.97));
    public static final Distance      TRENCH_LENGTH_Z = Inches.of(22.25); // height of the trench opening
    public static final Distance      TRENCH_LENGTH_Y = Inches.of(2*24.97); // the size of the opening (extent along y-axis of field)
    public static final Translation3d TRENCH_DIMENSION = new Translation3d(Inches.of(0), Inches.of(2*24.97), Inches.of(22.25));

    public static final Translation2d BLUE_RAMP_RIGHT_CENTER = new Translation2d(in2m(181.56), in2m(24.97*2+12+73/2.0));
    public static final Translation2d BLUE_RAMP_LEFT_CENTER = new Translation2d(in2m(181.56), in2m(FIELD_LENGTH_Y.in(Inches)-(24.97*2+12+73/2.0)));
    public static final Distance      RAMP_LENGTH_X = Inches.of(44.4); // extent along the x-axis of the field
    public static final Distance      RAMP_LENGTH_Y = Inches.of(73); // extent along the y-axis of the field
    public static final Distance      RAMP_LENGTH_Z = Inches.of(6.513); // ramp starts at 0 height and goes to this height in the middle
    public static final Translation3d RAMP_DIMENSION = new Translation3d(Inches.of(44.4), Inches.of(73), Inches.of(6.513));


    // contains all the AprilTags in wpiBlue coordinates
    // There are 32 of them with IDs 1-32
    // access them like this:
    // List<AprilTag> tags = ATF.getTags(); // gets a list of all the april tags as AprilTag classes (should contain 32 of them)
    // Pose3d pose = ATF.getTagPose(10); // gets the pose directly of the tag with ID 10
    public static final AprilTagFieldLayout ATF = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
}
