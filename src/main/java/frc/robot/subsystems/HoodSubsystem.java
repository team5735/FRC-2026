package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SingleSubsystem;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.util.NTable;

public class HoodSubsystem extends SubsystemBase {
    public final Trigger exclusionZoneTrigger = new Trigger(this::isInExclusionZone);

    private final Servo servo = new Servo(Constants.HOOD_SERVO_PIN);
    private final AnalogInput feedback = new AnalogInput(Constants.HOOD_FEEDBACK_PIN);

    private Supplier<Pose2d> turretPoseSupplier;
    private Rectangle2d[] exclusionZones;
    private double exclusionZoneSavedServoPosition;

    private InterpolatingDoubleTreeMap hoodToServoPosition = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap servoToHoodPosition = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap angleToServoPosition = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap servoToAnglePosition = new InterpolatingDoubleTreeMap();

    public HoodSubsystem(Supplier<Pose2d> turretPoseSupplier, Rectangle2d[] exclusionZones) {
        this.turretPoseSupplier = turretPoseSupplier;
        this.exclusionZones = exclusionZones;

        this.hoodToServoPosition.put(0.0, HoodConstants.LOWEST_SERVO_POSITION);
        this.hoodToServoPosition.put(1.0, HoodConstants.HIGHEST_SERVO_POSITION);
        this.servoToHoodPosition.put(HoodConstants.LOWEST_SERVO_POSITION, 0.0);
        this.servoToHoodPosition.put(HoodConstants.HIGHEST_SERVO_POSITION, 1.0);

        this.angleToServoPosition.put(HoodConstants.LOWEST_ANGLE_DEGREES, HoodConstants.LOWEST_SERVO_POSITION);
        this.angleToServoPosition.put(HoodConstants.HIGHEST_ANGLE_DEGREES, HoodConstants.HIGHEST_SERVO_POSITION);
        this.servoToAnglePosition.put(HoodConstants.LOWEST_SERVO_POSITION, HoodConstants.LOWEST_ANGLE_DEGREES);
        this.servoToAnglePosition.put(HoodConstants.HIGHEST_SERVO_POSITION, HoodConstants.HIGHEST_ANGLE_DEGREES);
    }

    public double getServoSetpoint() {
        return servo.get();
    }

    public void setServoPosition(double pos) {
        // todo: should log warning if incoming pos is out of range
        pos = MathUtil.clamp(pos, 0.0, 1.0);
        this.servo.set(pos);
        this.sendTelemetry();
    }

    public double getHoodPosition() {
        return this.servoToHoodPosition.get(this.getServoSetpoint());
    }

    public void setHoodPosition(double hoodPosition) {
        double servoPosition = this.hoodToServoPosition.get(hoodPosition);
        this.setServoPosition(servoPosition);
    }

    public double getHoodAngle() {
        return this.servoToAnglePosition.get(this.getServoSetpoint());
    }

    public void setHoodAngle(double hoodAngleDegrees) {
        double servoPosition = this.angleToServoPosition.get(hoodAngleDegrees);
        this.setServoPosition(servoPosition);
    }

    public void exzSaveServoPosition() {
        this.exclusionZoneSavedServoPosition = this.servo.get();
    }

    public double exzGetSavedServoPosition() {
        return exclusionZoneSavedServoPosition;
    }

    // Returns raw voltage from analog feedback wire
    public double getVoltage() {
        return feedback.getVoltage();
    }

    // Converts voltage (0-5V) into 0.0–1.0 normalized position
    public double getNormalizedPosition() {
        return feedback.getVoltage() / 5.0;
    }

    public void sendTelemetry() {
        SmartDashboard.putNumber("hood/hood_position", this.getHoodPosition());
        SmartDashboard.putNumber("hood/servo_position", this.getServoSetpoint());
        SmartDashboard.putNumber("hood/servo_feedback_voltage", this.getVoltage());
        SmartDashboard.putNumber("hood/servo_feedback_position", getNormalizedPosition());
        SmartDashboard.putNumber("hood/hood_angle_degrees", this.getHoodAngle());
    }

    public boolean isInExclusionZone() {
        for (Rectangle2d r : exclusionZones) {
            if (r.contains(turretPoseSupplier.get().getTranslation()))
                return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        this.sendTelemetry();
    }

    // This is a full robot config for testing the hood subsystem
    public static class Tester extends SingleSubsystem {
        private final HoodSubsystem hood = new HoodSubsystem(() -> new Pose2d(),
                FieldConstants.HOOD_EXCLUSION_ZONES);

        private double lastPos = 0.5;

        public Tester() {
            super();

            controller.y().onTrue(hood.runOnce(() -> hood.setHoodPosition(1.0)));
            controller.b().onTrue(hood.runOnce(() -> hood.setHoodPosition(0.0)));

            controller.x().onTrue(hood.runOnce(() -> {
                lastPos += 0.025;
                lastPos = MathUtil.clamp(lastPos, 0.0, 1.0);
                hood.setServoPosition(lastPos);
            }));
            controller.a().onTrue(hood.runOnce(() -> {
                lastPos -= 0.025;
                lastPos = MathUtil.clamp(lastPos, 0.0, 1.0);
                hood.setServoPosition(lastPos);
            }));
        }
    };

    // test hood up/down in exclusion zones
    // move trench april tag closer / further from unmoving bot
    // to trigger response
    public static class PeekABooBot extends SingleSubsystem {
        private Field2d field2d = new Field2d();
        private final HoodSubsystem hood = new HoodSubsystem(() -> field2d.getRobotPose(),
                FieldConstants.HOOD_EXCLUSION_ZONES);

        public PeekABooBot() {
            super();

            NTable.root("SmartDashboard").sub("hood").set("draggable robot for peek-a-bot", field2d);

            hood.setHoodPosition(0.4);

            hood.exclusionZoneTrigger.onTrue(Commands.runOnce(() -> {
                SmartDashboard.putBoolean("hood/in_exclusion_zone", true);
                hood.exzSaveServoPosition();
                hood.setHoodPosition(0);
            }));
            hood.exclusionZoneTrigger.onFalse(Commands.runOnce(() -> {
                SmartDashboard.putBoolean("hood/in_exclusion_zone", false);
                double pos = hood.exzGetSavedServoPosition();
                hood.setServoPosition(pos);
            }));
        }

        @Override
        public void robotPeriodic() {
            super.robotPeriodic();
        }

    };
}
