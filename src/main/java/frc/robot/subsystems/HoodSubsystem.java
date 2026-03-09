package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.PartialRobot;
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

    /**
     * Linear interpolation. Returns the interpolated value of query point xq
     * against the line
     * defined by (x0, y0) and (x1, y1).
     * xq should be between x0 and x1. The value returned is "yq"
     * If xq is outside x0 and x1, the value will be clamped to y0 and y1
     * respectively.
     */

    public static double interp1(double x0, double x1, double y0, double y1, double xq) {
        if (x0 == x1) {
            throw new IllegalArgumentException("x1 and x2 cannot be equal for interpolation");
        }
        if (xq >= x1)
            return y1;
        if (xq <= x0)
            return y0;

        return y0 + (xq - x0) * (y1 - y0) / (x1 - x0);
    }

    public HoodSubsystem(Supplier<Pose2d> turretPoseSupplier, Rectangle2d[] exclusionZones) {
        super();
        this.turretPoseSupplier = turretPoseSupplier;
        this.exclusionZones = exclusionZones;
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
        return interp1(
                HoodConstants.LOWEST_SERVO_POSITION, 0.0,
                HoodConstants.HIGHEST_SERVO_POSITION, 1.0,
                this.getServoSetpoint());
    }

    public void setHoodPosition(double hoodPosition) {
        double servoPosition = interp1(
                0, HoodConstants.LOWEST_SERVO_POSITION,
                1, HoodConstants.HIGHEST_SERVO_POSITION,
                hoodPosition);
        this.setServoPosition(servoPosition);
    }

    public double getHoodAngle() {
        return interp1(
                HoodConstants.LOWEST_SERVO_POSITION, HoodConstants.LOWEST_ANGLE_DEGREES,
                HoodConstants.HIGHEST_SERVO_POSITION, HoodConstants.HIGHEST_ANGLE_DEGREES,
                this.getServoSetpoint());
    }

    public void setHoodAngle(double hoodAngleDegrees) {
        double servoPosition = interp1(
                HoodConstants.LOWEST_ANGLE_DEGREES, HoodConstants.LOWEST_SERVO_POSITION,
                HoodConstants.HIGHEST_ANGLE_DEGREES, HoodConstants.HIGHEST_SERVO_POSITION,
                hoodAngleDegrees);

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
        double v = this.getVoltage();
        return interp1(HoodConstants.SERVO_VOLTAGE_ZERO_SETPOINT, 0.0,
                HoodConstants.SERVO_VOLTAGE_ONE_SETPOINT, 1.0,
                v);
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

    public Command getDynamicTracking(Supplier<Angle> angleSupplier) {
        return run(() -> setHoodAngle(angleSupplier.get().in(Degrees)));
    }

    @Override
    public void periodic() {
        this.sendTelemetry();
    }

    // This is a full robot config for testing the hood subsystem
    public static class Tester extends PartialRobot {
        private final HoodSubsystem hood = new HoodSubsystem(() -> new Pose2d(),
                FieldConstants.HOOD_EXCLUSION_ZONES);

        private double lastPos = 0.6;

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

        @Override
        public void robotPeriodic() {
            this.hood.sendTelemetry();
            super.robotPeriodic();
        }

    };

    // test hood up/down in exclusion zones
    // move trench april tag closer / further from unmoving bot
    // to trigger response
    public static class PeekABooBot extends PartialRobot {
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
            this.hood.sendTelemetry();
            super.robotPeriodic();
        }

    };
}
