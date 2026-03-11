package frc.robot.util.geometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.util.NTable;

public class Arc {
    Translation2d center;
    double radius;
    Rotation2d start, delta;

    NTable table = NTable.root("arc").sub("debugging");

    // delta must be positive
    public Arc(Translation2d center, double radius, Rotation2d start, Rotation2d delta) {
        this.center = center;
        this.radius = radius;
        this.start = start;
        this.delta = delta;
    }

    private double mod(double in) {
        return MathUtil.inputModulus(in, 0, 2 * Math.PI);
    }

    private Rotation2d mod(Rotation2d in) {
        return Rotation2d.fromRadians(mod(in.getRadians()));
    }

    public boolean angleInRange(Rotation2d theta_) {
        double theta = mod(theta_.getRadians());
        double first = mod(start.getRadians());
        double second = mod(start.plus(delta).getRadians());

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
        table.set("end", delta.getDegrees());
        if (angleInRange(thetaA)) {
            table.set("in range", true);
            return new Translation2d(radius, thetaA).plus(center);
        }
        table.set("in range", false);

        Translation2d p1 = center.plus(new Translation2d(radius, start));
        Translation2d p2 = center.plus(new Translation2d(radius, start.plus(delta)));

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

    public Rotation2d getDelta() {
        return delta;
    }

    public Rotation2d getEnd() {
        return start.plus(delta);
    }

    public Pose2d[] getAsPoses() {
        Pose2d[] result = new Pose2d[100];
        for (int i = 0; i < 100; i++) {
            Rotation2d theta = start.interpolate(start.plus(delta), i / (100 - 1.0));
            result[i] = new Pose2d(center.plus(new Translation2d(radius, theta)), Rotation2d.kZero);
        }
        return result;
    }

    public Arc alliance() {
        Rotation2d start = this.start;
        Rotation2d delta = this.delta;
        if (FieldConstants.shouldSwitchAlliance()) {
            start = mod(Rotation2d.k180deg.plus(start));
        }
        return new Arc(FieldConstants.alliance(this.center), this.radius, start, delta);
    }
}
