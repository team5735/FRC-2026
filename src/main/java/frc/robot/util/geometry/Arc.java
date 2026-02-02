package frc.robot.util.geometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public void telemeterize(Pose2d robotPose) {
        System.out.println("telemeterizing arc");
        Field2d field = new Field2d();
        field.setRobotPose(robotPose);
        field.getObject("center").setPose(new Pose2d(center, Rotation2d.kZero));
        Translation2d startPos = new Translation2d(radius, start).plus(center);
        field.getObject("start").setPose(new Pose2d(startPos, startPos.minus(center).getAngle()));
        Translation2d endPos = new Translation2d(radius, end).plus(center);
        field.getObject("end").setPose(new Pose2d(endPos, endPos.minus(center).getAngle()));
        Translation2d nearest = nearestPointOnArc(robotPose.getTranslation());
        field.getObject("nearest").setPose(new Pose2d(nearest, nearest.minus(center).getAngle()));
        SmartDashboard.putData("arc as a field", field);
    }
}
