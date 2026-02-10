package frc.robot.util.geometry;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.util.NTable;

public class Arc {
    Translation2d center;
    double radius;
    Rotation2d start, end;

    public Arc(Translation2d center, double radius, Rotation2d start, Rotation2d end) {
        this.center = center;
        this.radius = radius;
        this.start = start;
        this.end = end;
    }

    public boolean angleInRange(Rotation2d theta_) {
        double theta = MathUtil.inputModulus(theta_.getRadians(), 0, 2 * Math.PI);
        double first = MathUtil.inputModulus(start.getRadians(), 0, 2 * Math.PI);
        double second = MathUtil.inputModulus(end.getRadians(), 0, 2 * Math.PI);

        if (first > second) {
            return theta >= second && theta <= first;
        }
        return theta >= first && theta <= second;
    }

    public Translation2d nearestPointOnArc(Translation2d position) {
        // return any point on the arc if we're passed the center
        if (position.equals(center)) {
            return center.plus(new Translation2d(radius, start));
        }

        Translation2d centerToGiven = position.minus(center);
        Rotation2d thetaA = centerToGiven.getAngle();

        if (angleInRange(thetaA)) {
            return new Translation2d(radius, thetaA).plus(center);
        }

        Translation2d p1 = center.plus(new Translation2d(radius, start));
        Translation2d p2 = center.plus(new Translation2d(radius, end));

        return position.getDistance(p1) <= position.getDistance(p2) ? p1 : p2;
    }

    public Pose2d getPoseFacingCenter(Translation2d position) {
        return new Pose2d(position, position.minus(center).getAngle());
    }

    public Translation2d getCenter() {
        return center;
    }

    public double getRadius() {
        return radius;
    }

    public Rotation2d getStart() {
        return start;
    }

    public Rotation2d getEnd() {
        return end;
    }

    public void telemeterize(Pose2d robotPose) {
        Field2d field = new Field2d();
        field.setRobotPose(robotPose);
        field.getObject("center").setPose(new Pose2d(center, start.interpolate(end, 0.5)));
        int numPoints = 100;
        List<Pose2d> points = new ArrayList<>(numPoints);
        for (int i = 0; i < numPoints; i++) {
            Rotation2d theta = start.interpolate(end, i / (numPoints - 1.0));
            points.add(new Pose2d(center.plus(new Translation2d(radius, theta)), Rotation2d.kZero));
        }
        field.getObject("arc").setPoses(points);
        Translation2d nearest = nearestPointOnArc(robotPose.getTranslation());
        field.getObject("nearest").setPose(new Pose2d(nearest, nearest.minus(center).getAngle()));
        NTable.root("arc").setSendable("as a field", field);
    }
}
