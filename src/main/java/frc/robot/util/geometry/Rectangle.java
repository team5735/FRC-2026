package frc.robot.util.geometry;

import java.awt.geom.Rectangle2D;

import edu.wpi.first.math.geometry.Translation2d;

public class Rectangle extends Rectangle2D.Double {
    public Rectangle(Translation2d corner, Translation2d dimension) {
        super(corner.getX(), corner.getY(), dimension.getX(), dimension.getY());
    }

    public Translation2d getCenter() {
        return new Translation2d(getCenterX(), getCenterY());
    }

    public Translation2d getDimensions() {
        return new Translation2d(getWidth(), getHeight());
    }

    public Translation2d getPos() {
        return new Translation2d(getX(), getY());
    }

    public Translation2d getUpperRight() {
        return getPos().plus(getDimensions());
    }

    public boolean contains(Translation2d point) {
        return contains(point.getX(), point.getY());
    }
}
