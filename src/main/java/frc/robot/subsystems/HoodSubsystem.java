package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodSubsystem extends SubsystemBase {
    private final Servo servo = new Servo(0);
    private final AnalogInput feedback = new AnalogInput(0);
    private Supplier<Pose2d> turretPoseSupplier;
    private Rectangle2d[] exclusionZones;
    public final Trigger exclusionZoneTrigger = new Trigger(this::isInExclusionZone);

    // Runs one time when the robot starts
    public HoodSubsystem(Supplier<Pose2d> turretPoseSupplier, Rectangle2d[] exclusionZones) {
        this.exclusionZones=exclusionZones;
        this.turretPoseSupplier=turretPoseSupplier;
        SmartDashboard.putNumber("Start Revolution Posiiton", Constants.START_REVOLUTION_POSITION);
        SmartDashboard.putNumber("End Revolution Posiiton", Constants.END_REVOLUTION_POSITION);
        setPosition(Constants.START_REVOLUTION_POSITION); // default safe position at startup
    }

    // Pos is set anywhere from 0 to 1
    public void setPosition(double position) {
        double safePosition = Math.max(0.1, Math.min(0.9, position));
        servo.set(safePosition);
    }

    // Returns raw voltage from analog feedback wire
    public double getVoltage() {
        return feedback.getVoltage();
    }

    // Converts voltage (0-5V) into 0.0–1.0 normalized position
    public double getNormalizedPosition() {
        return feedback.getVoltage() / 5.0;
    }

    public boolean isInExclusionZone(){
    
        for (Rectangle2d r: exclusionZones){
            if (r.contains(turretPoseSupplier.get().getTranslation())) 
                return true;
        }
        return false;

    }

    // Runs every 20ms automatically
    @Override
    public void periodic() {

    }
}
