package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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
        double theta = MathUtil.angleModulus(theta_.getRadians());
        double first = MathUtil.angleModulus(start.getRadians());
        double second = MathUtil.angleModulus(end.getRadians());

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
}
