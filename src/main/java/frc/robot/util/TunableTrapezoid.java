package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TunableTrapezoid {
    protected TrapezoidProfile profile;

    private NTable table;

    public TunableTrapezoid(String name) {
        this.table = NTable.root("trapezoid");
    }

    public void setup() {
        double maxVelocity = this.table.getDouble("max velocity");
        double maxAcceleration = this.table.getDouble("max acceleration");
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }

    public double 
}
