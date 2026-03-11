package frc.robot.util.geometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.util.NTable;

public class Arc {
    Translation2d center;
    double radius;
    Rotation2d start, end;

    NTable table = NTable.root("arc").sub("debugging");

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

        table.set("theta wrapped", theta);
        table.set("first wrapped", first);
        table.set("second wrapped", second);
        if (first <= second) {
            return theta >= first && theta <= second;
        }
        return theta >= first || theta <= second;
    }

    public Translation2d nearestPointOnArc(Translation2d position) {
        // return any point on the arc if we're passed the center
        if (position.equals(center)) {
            return center.plus(new Translation2d(radius, start));
        }

        Translation2d centerToGiven = position.minus(center);
        Rotation2d thetaA = centerToGiven.getAngle();

        table.set("thetaA", thetaA.getDegrees());
        table.set("start", start.getDegrees());
        table.set("end", end.getDegrees());
        if (angleInRange(thetaA)) {
            table.set("in range", true);
            return new Translation2d(radius, thetaA).plus(center);
        }
        table.set("in range", false);

        Translation2d p1 = center.plus(new Translation2d(radius, start));
        Translation2d p2 = center.plus(new Translation2d(radius, end));

        return position.getDistance(p1) <= position.getDistance(p2) ? p1 : p2;
    }

    public Pose2d getPoseFacingCenter(Translation2d position) {
        return new Pose2d(position, center.minus(position).getAngle());
    }

    public Pose2d getShootingPose(Translation2d pose, Rotation2d offset) {
        Pose2d facingCenterOnArc = getPoseFacingCenter(nearestPointOnArc(pose));
        return new Pose2d(facingCenterOnArc.getTranslation(), facingCenterOnArc.getRotation().plus(offset));
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

    public Pose2d[] getAsPoses() {
        Pose2d[] result = new Pose2d[100];
        for (int i = 0; i < 100; i++) {
            Rotation2d theta = start.interpolate(end, i / (100 - 1.0));
            result[i] = new Pose2d(center.plus(new Translation2d(radius, theta)), Rotation2d.kZero);
        }
        return result;
    }

    public Arc alliance() {
        boolean shouldFlip = DriverStation.getAlliance().get() == Alliance.Red;
        return new Arc(
                FieldConstants.alliance(this.center),
                this.radius,
                this.end,
                this.start.plus(shouldFlip ? Rotation2d.k180deg : Rotation2d.kZero));
    }
}
