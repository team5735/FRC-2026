package frc.robot.util.geometry;

import edu.wpi.first.math.geometry.Translation2d;

public class Rectangle {
    private Translation2d center;
    private Translation2d dim;

    private double xmin, xmax, ymin, ymax;

    public Rectangle(Translation2d center, Translation2d dimension){
        this.center = center;
        this.dim = dimension;

        this.xmin = this.center.getX() - this.dim.getX() / 2.0;
        this.xmax = this.center.getX() + this.dim.getX() / 2.0;
        this.ymin = this.center.getY() - this.dim.getY() / 2.0;
        this.ymax = this.center.getY() + this.dim.getY() / 2.0;
    }

    public Translation2d getCenter(){ return center;};
    public Translation2d getDimension(){ return dim;};
    public Translation2d getUpperRight(){ 
        return new Translation2d(center.getX()+dim.getX()/2,
                                 center.getY()+dim.getY()/2);
    }
    public Translation2d getLowerLeft(){ 
        return new Translation2d(center.getX()-dim.getX()/2,
                                 center.getY()-dim.getY()/2);
    }
    

    @Override
    public String toString() {
        return String.format("Rectangle(Center: %s, Dimension: %s", this.center, this.dim);
    }


    public boolean within(Translation2d testPt){
        double x = testPt.getX();
        double y = testPt.getY();
        return x >= this.xmin && x <= this.xmax && y >= this.ymin && y <= this.ymax;

    }
}
