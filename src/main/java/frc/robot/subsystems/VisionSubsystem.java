// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NTDoubleSection;

public class VisionSubsystem extends SubsystemBase {
    DrivetrainSubsystem drivetrain;
    @SuppressWarnings("unused")
    private double driftEstimateTicks;

    public static final String LIMELIGHTS[] = { "limelight-left" };

    private NTDoubleSection doubles = new NTDoubleSection("vision", "drivetrainYaw", "drivetrainOmegaZ",
            "drivetrainYaw");

    public VisionSubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        for (String limelight : LIMELIGHTS) {
            doubles.addEntry(limelight + "_status");
        }
    }

    private boolean trySeedPigeon(String name) {
        if (!LimelightHelpers.getTV(name)) {
            return false;
        }
        drivetrain.getPigeon2()
                .setYaw(LimelightHelpers.getBotPoseEstimate_wpiBlue(name).pose.getRotation().getDegrees());
        drivetrain.resetPose(
                new Pose2d(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose.getTranslation(),
                        drivetrain.getPigeon2().getRotation2d()));
        return true;
    }

    public void seedPigeon() {
        for (String limelight : LIMELIGHTS) {
            trySeedPigeon(limelight);
        }
    }

    public record RawFiducial(double ambiguity, Distance distToCamera) {
    }

    private class PoseEstimate {
        RawFiducial[] fiducials;
        Pose2d pose2d;
        Pose3d pose3d;
        double timestamp;
        Pose3d stddevs;
        double distToCamera;

        public PoseEstimate(String limelightName, boolean isMT1) {
            double[] stddevs = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("stddevs")
                    .getDoubleArray(new double[0]);
            TimestampedDoubleArray atomicArray;
            if (isMT1) {
                atomicArray = NetworkTableInstance.getDefault().getTable(limelightName)
                        .getDoubleArrayTopic("botpose_wpiblue").getEntry(new double[0]).getAtomic();
            } else {
                LimelightHelpers.SetRobotOrientation(limelightName, drivetrain.getPigeon2().getYaw().getValueAsDouble(),
                        0, 0, 0, 0, 0);
                atomicArray = NetworkTableInstance.getDefault().getTable(limelightName)
                        .getDoubleArrayTopic("botpose_orb_wpiblue").getEntry(new double[0]).getAtomic();
            }

            double[] array = atomicArray.value;
            Translation3d translation = new Translation3d(array[0], array[1], array[2]);
            Rotation3d rotation = new Rotation3d(array[3], array[4], array[5]);
            double latency = array[6];

            int nFiducials = (int) array[7];
            RawFiducial[] fiducials = new RawFiducial[nFiducials];
            for (int i = 0; i < nFiducials; i++) {
                double[] fiducial = Arrays.copyOfRange(array, 11 + 7 * i, 18 + 7 * i);
                fiducials[i] = new RawFiducial(fiducial[6], Meters.of(fiducial[4]));
            }

            this.pose3d = new Pose3d(translation, rotation);
            this.pose2d = pose3d.toPose2d();
            this.timestamp = atomicArray.timestamp - latency;
            this.fiducials = fiducials;
            this.distToCamera = array[9];

            if (!isMT1) {
                // mt2 is the same format as mt1 data just in the second half of the array; copy
                // the second half to the first
                System.arraycopy(array, 6, array, 0, 6);
            }
            this.stddevs = new Pose3d(new Translation3d(stddevs[0], stddevs[1], stddevs[2]),
                    new Rotation3d(stddevs[3], stddevs[4], stddevs[5]));
        }
    }

    private double lastPigeonReset = 0;

    private void maybeResetPigeon(String limelightName, PoseEstimate estimate) {
        // obey the minimum reset delay
        if (Timer.getFPGATimestamp() - lastPigeonReset < VisionConstants.MIN_RESET_DELAY.in(Seconds)) {
            SmartDashboard.putString(limelightName + " reset status", "too soon since last reset");
            return;
        }

        // stay grounded to reality!
        if (estimate.pose3d.getMeasureZ().abs(Meters) > VisionConstants.TOLERATED_HEIGHT.in(Meters)) {
            SmartDashboard.putString(limelightName + " reset status",
                    "mt1 pose estimate is more than 3cm away from the ground");
            return;
        }

        // limit angular velocity
        if (drivetrain.getPigeon2().getAngularVelocityZWorld().asSupplier().get()
                .in(DegreesPerSecond) < VisionConstants.MAX_ANGULAR_VELOCITY_FOR_RESET.in(DegreesPerSecond)) {
            SmartDashboard.putString(limelightName + " reset status", "robot is rotating");
            return;
        }

        // all pose estimate ambiguity < 0.2
        if (Arrays.stream(estimate.fiducials)
                .anyMatch(tag -> tag.ambiguity > VisionConstants.MAX_AMBIGUITY_FOR_RESET)) {
            SmartDashboard.putString(limelightName + " reset status", "any tag ambiguity too high");
            return;
        }

        // at least one tag is close (1.5m)
        if (Arrays.stream(estimate.fiducials)
                .noneMatch(tag -> tag.distToCamera.in(Meters) < VisionConstants.NEAR_ENOUGH_TO_RESET.in(Meters))) {
            SmartDashboard.putString(limelightName + " reset status", "no tag close enough to reset");
            return;
        }

        // yaw stddev < 5 degrees
        if (estimate.stddevs.getRotation().getMeasureZ().in(Degrees) > VisionConstants.MAX_YAW_STDDEV_FOR_RESET
                .in(Degrees)) {
            SmartDashboard.putString(limelightName + " reset status", "yaw stddev too high");
            return;
        }

        SmartDashboard.putString(limelightName + " reset status", "accepted!");
        System.out.println("resetting pigeon from mt1 from " + limelightName);
        drivetrain.getPigeon2().setYaw(estimate.pose2d.getRotation().getMeasure());
        lastPigeonReset = Timer.getFPGATimestamp();
    }

    public void handleVisionMeasurement(String limelightName) {
        PoseEstimate mt1 = new PoseEstimate(limelightName, true);
        PoseEstimate mt2 = new PoseEstimate(limelightName, false);
        doubles.set("pigeon reset time", lastPigeonReset);
        maybeResetPigeon(limelightName, mt1);

        // mt1 and mt2 differ significantly
        if (mt1.pose3d.getTranslation().getDistance(
                mt2.pose3d.getTranslation()) > VisionConstants.MAX_DISTANCE_BETWEEN_MT1_AND_MT2.in(Meters)) {
            SmartDashboard.putString(limelightName + " status", "mt1 and mt2 pose estimates differ significantly");
            return;
        }

        // mt1 pose estimate is more than 3cm off the ground
        if (mt1.pose3d.getMeasureZ().abs(Meters) > VisionConstants.TOLERATED_HEIGHT.in(Meters)) {
            SmartDashboard.putString(limelightName + " status",
                    "mt1 pose estimate is more than 3cm away from the ground");
            return;
        }

        // mt1 pose estimate is off the field
        double robotBoundingBox = Math.max(
                DrivetrainSubsystem.CONSTANTS.getRobotTotalLength().in(Meters),
                DrivetrainSubsystem.CONSTANTS.getRobotTotalWidth().in(Meters)) * Math.sqrt(2);
        if (mt1.pose2d.getTranslation().getX() < robotBoundingBox / 2
                || mt1.pose2d.getTranslation()
                        .getX() > (FieldConstants.FIELD_LENGTH_X.in(Meters) - robotBoundingBox / 2)
                || mt1.pose2d.getTranslation().getY() < robotBoundingBox / 2
                || mt1.pose2d.getTranslation()
                        .getY() > (FieldConstants.FIELD_LENGTH_Y.in(Meters) - robotBoundingBox / 2)) {
            SmartDashboard.putString(limelightName + " status", "mt1 pose estimate is off the field");
            return;
        }

        // 1 tag pose estimate and ambiguity > 0.2
        if (mt1.fiducials.length == 1 && mt1.fiducials[0].ambiguity > VisionConstants.SINGLE_TAG_MAX_AMBIGUITY) {
            SmartDashboard.putString(limelightName + " status", "1 tag ambiguity > 0.2");
            return;
        }

        // multi tag pose estimate and any tag ambiguity > 0.5
        if (mt1.fiducials.length > 1 && Arrays.stream(mt1.fiducials)
                .anyMatch(tag -> tag.ambiguity > VisionConstants.MULTI_TAG_MAX_AMBIGUITY)) {
            SmartDashboard.putString(limelightName + " status", "multi tag ambiguity > 0.5");
            return;
        }

        // robot is rotating (if we get angular velocity)
        if (drivetrain.getPigeon2().getAngularVelocityZWorld().asSupplier().get()
                .in(DegreesPerSecond) < VisionConstants.TOLERATED_ROTATIONAL_RATE.in(DegreesPerSecond)) {
            SmartDashboard.putString(limelightName + " status", "robot is rotating");
            return;
        }

        if (Timer.getFPGATimestamp() - lastPigeonReset < VisionConstants.MT2_DRIFT_TOLERANCE.in(Seconds)) {
            SmartDashboard.putString(limelightName + " status", "accepted! using mt2");
            updateVisionMeasurement(limelightName, mt2);
        } else {
            SmartDashboard.putString(limelightName + " status", "accepted! using mt1");
            updateVisionMeasurement(limelightName, mt1);
        }
    }

    private boolean drivetrainIsNaNOrInf() {
        return Double.isNaN(drivetrain.getEstimatedPosition().getX())
                || Double.isInfinite(drivetrain.getEstimatedPosition().getX()) ||
                Double.isNaN(drivetrain.getEstimatedPosition().getY())
                || Double.isInfinite(drivetrain.getEstimatedPosition().getY());
    }

    private void updateVisionMeasurement(String limelightName, PoseEstimate estimate) {
        // completely reset the pose estimator if it's not having a good time
        if (drivetrainIsNaNOrInf()) {
            System.out.println("resetting drivetrain pose estimator due to nan or inf");
            drivetrain.resetPose(estimate.pose2d);
        }

        Vector<N3> stddevs = VecBuilder.fill(estimate.stddevs.getX(), estimate.stddevs.getY(),
                estimate.stddevs.getRotation().getMeasureZ().in(Radians));

        // multiply stddevs by avg tag dist to camera: stddev*(1+dist)
        stddevs = stddevs.times(1 + estimate.distToCamera);
        // multiply by robot velocity stddev*(1+mps)
        // TODO: do if possible
        // if robot is rotating, use 99999 for theta
        if (drivetrain.getPigeon2().getAngularVelocityZWorld().asSupplier().get()
                .in(DegreesPerSecond) > VisionConstants.ROTATION_EPSILON.in(DegreesPerSecond)) {
            stddevs.getData()[2] = 99999;
        }

        // ensure reasonable minimums (x, y: 5cm, theta: 5°)
        stddevs.getData()[0] = Math.max(Centimeters.of(5).in(Meters), stddevs.getData()[0]);
        stddevs.getData()[1] = Math.max(Centimeters.of(5).in(Meters), stddevs.getData()[1]);
        stddevs.getData()[2] = Math.max(Degrees.of(5).in(Radians), stddevs.getData()[2]);

        drivetrain.addVisionMeasurement(estimate.pose2d, estimate.timestamp, stddevs);
    }

    @Override
    public void periodic() {
        for (String limelight : LIMELIGHTS) {
            handleVisionMeasurement(limelight);
        }
    }
}
