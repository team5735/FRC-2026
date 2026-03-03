package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;

import java.util.function.Supplier;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodSubsystem extends SubsystemBase {
    public final Trigger exclusionZoneTrigger = new Trigger(this::isInExclusionZone);

    private final Servo servo = new Servo(0);
    private final AnalogInput feedback = new AnalogInput(0);

    private Supplier<Pose2d> turretPoseSupplier;
    private Rectangle2d[] exclusionZones;
    private double exclusionZonesSavedPosition;

    private InterpolatingDoubleTreeMap hoodToServoPosition = new InterpolatingDoubleTreeMap();

    public HoodSubsystem(Supplier<Pose2d> turretPoseSupplier, Rectangle2d[] exclusionZones) {
        this.turretPoseSupplier = turretPoseSupplier;
        this.exclusionZones   = exclusionZones;

        this.hoodToServoPosition.put(0.0, Constants.HOOD_LOWEST_SERVO_POSITION);
        this.hoodToServoPosition.put(1.0, Constants.HOOD_HIGHEST_SERVO_POSITION);
    }

    public double getServoPosition(){
        return servo.get();
    }

    public void setServoPosition(double pos) {
        if (pos < 0 || pos > 1.0) {
            throw new IllegalArgumentException("pos " + pos + " not between 0-1");
        }
        this.servo.set(pos);
    }

    public void setHoodPosition(double hoodPosition) {
        double servoPosition = this.hoodToServoPosition.get(hoodPosition);
        servo.set(servoPosition);
    }

    public void setAndSavePosition(double hoodPosition) {
        exclusionZonesSavedPosition = getServoPosition();
        setHoodPosition(hoodPosition);
    }

    public double getExclusionZonesSavedPosition() {
        return exclusionZonesSavedPosition;
    }

    // Returns raw voltage from analog feedback wire
    public double getVoltage() {
        return feedback.getVoltage();
    }

    // Converts voltage (0-5V) into 0.0–1.0 normalized position
    public double getNormalizedPosition() {
        return feedback.getVoltage() / 5.0;
    }

    public boolean isInExclusionZone() {

        for (Rectangle2d r : exclusionZones) {
            if (r.contains(turretPoseSupplier.get().getTranslation()))
                return true;
        }
        return false;
    }

    // Runs every 20ms automatically
    @Override
    public void periodic() {
        if (Constants.HOOD_TUNING_MODE || Constants.BREADBOARD_MODE) {
            SmartDashboard.putNumber("Servo Position Degrees", (int) (feedback.getVoltage() / 5. * 1800));
        }
    }
}
