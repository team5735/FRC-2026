// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NTable;

public class VisionSubsystem extends SubsystemBase {
    DrivetrainSubsystem drivetrain;
    @SuppressWarnings("unused")
    private double driftEstimateTicks;

    private NTable table = NTable.root("vision");

    public VisionSubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        this.table.set("enabled", true);
        this.table.makePersistent("enabled");
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
        for (String limelight : VisionConstants.LIMELIGHTS) {
            trySeedPigeon(limelight);
        }
    }

    public record RawFiducial(double ambiguity, Distance distToCamera) {
    }

    public class PoseEstimate {
        public RawFiducial[] fiducials;
        public Pose2d pose2d;
        public Pose3d pose3d;
        public double timestamp;
        public Pose3d stddevs;
        public double distToCamera;

        private static Map<String, DoubleArrayEntry> entriesCache = new HashMap<>();

        private DoubleArrayEntry getDoubleArrayEntry(String table, String topic) {
            String asPath = table + "/" + topic;
            return entriesCache.computeIfAbsent(asPath, k -> {
                System.out.println("creating subscriber for " + asPath);
                return NetworkTableInstance.getDefault().getTable(table)
                        .getDoubleArrayTopic(topic).getEntry(new double[0]);
            });
        }

        public PoseEstimate(String limelightName) {
            double[] stddevs = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("stddevs")
                    .getDoubleArray(new double[0]);
            TimestampedDoubleArray atomicArray;
            atomicArray = getDoubleArrayEntry(limelightName, "botpose_wpiblue").getAtomic();

            double[] array = atomicArray.value;
            if (array.length == 0) {
                return;
            }
            int nFiducials = (int) array[7];
            if (nFiducials == 0) {
                return;
            }
            Translation3d translation = new Translation3d(Meters.of(array[0]),
                    Meters.of(array[1]),
                    Meters.of(array[2]));
            Rotation3d rotation = new Rotation3d(Degrees.of(array[3]),
                    Degrees.of(array[4]),
                    Degrees.of(array[5]));
            double latency = array[6];

            RawFiducial[] fiducials = new RawFiducial[nFiducials];
            for (int i = 0; i < nFiducials; i++) {
                double[] fiducial = Arrays.copyOfRange(array, 11 + 7 * i, 18 + 7 * i);
                fiducials[i] = new RawFiducial(fiducial[6], Meters.of(fiducial[4]));
            }

            this.pose3d = new Pose3d(translation, rotation);
            this.pose2d = pose3d.toPose2d();
            this.timestamp = (atomicArray.timestamp / 1e6) - (latency / 1e3);
            this.fiducials = fiducials;
            this.distToCamera = array[9];

            this.stddevs = new Pose3d(
                    new Translation3d(Meters.of(stddevs[0]),
                            Meters.of(stddevs[1]),
                            Meters.of(stddevs[2])),
                    new Rotation3d(Degrees.of(stddevs[3]),
                            Degrees.of(stddevs[4]),
                            Degrees.of(stddevs[5])));
        }
    }

    public void handleVisionMeasurement(String limelightName) {
        NTable lltable = this.table.sub(limelightName);

        if (!this.table.getBoolean("enabled")) {
            lltable.set("status", "disabled in network tables");
            return;
        }

        PoseEstimate mt1 = new PoseEstimate(limelightName);
        if (mt1.pose2d == null) {
            lltable.set("status", "pose estimate is null");
            return;
        }

        Telemetry.field.getObject(limelightName).setPose(mt1.pose2d);

        lltable.set("dist to ground", mt1.pose3d.getMeasureZ().abs(Meters));
        // mt1 pose estimate is more than 3cm off the ground
        if (mt1.pose3d.getMeasureZ().abs(Meters) > VisionConstants.TOLERATED_HEIGHT.in(Meters)) {
            lltable.set("status", "pose estimate is more than 3cm away from the ground");
            return;
        }

        // mt1 pose estimate is off the field
        double conservativeRobotRadius = Math.max(
                DrivetrainSubsystem.CONSTANTS.getRobotTotalLength().in(Meters),
                DrivetrainSubsystem.CONSTANTS.getRobotTotalWidth().in(Meters)) * Math.sqrt(2);
        if (mt1.pose2d.getTranslation().getX() < conservativeRobotRadius / 2
                || mt1.pose2d.getTranslation()
                        .getX() > (FieldConstants.FIELD_LENGTH_X.in(Meters) - conservativeRobotRadius / 2)
                || mt1.pose2d.getTranslation().getY() < conservativeRobotRadius / 2
                || mt1.pose2d.getTranslation()
                        .getY() > (FieldConstants.FIELD_LENGTH_Y.in(Meters) - conservativeRobotRadius / 2)) {
            lltable.set("status", "pose estimate is off the field");
            return;
        }

        double[] ambiguities = Arrays.stream(mt1.fiducials).mapToDouble(tag -> tag.ambiguity).toArray();
        lltable.set("multi tag pose ambiguity", ambiguities);
        // 1 tag pose estimate and ambiguity > 0.2
        if (mt1.fiducials.length == 1 && mt1.fiducials[0].ambiguity > VisionConstants.SINGLE_TAG_MAX_AMBIGUITY) {
            lltable.set("status", "one tag; ambiguity > 0.2");
            return;
        }

        // multi tag pose estimate and any tag ambiguity > 0.5
        if (mt1.fiducials.length > 1 && Arrays.stream(mt1.fiducials)
                .anyMatch(tag -> tag.ambiguity > VisionConstants.MULTI_TAG_MAX_AMBIGUITY)) {
            lltable.set("status", "multi tag; any ambiguity > 0.5");
            return;
        }

        lltable.set("robot angular velocity", drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
        // robot is rotating (if we get angular velocity)
        if (drivetrain.getPigeon2().getAngularVelocityZWorld()
                .getValueAsDouble() > VisionConstants.TOLERATED_ROTATIONAL_RATE.in(DegreesPerSecond)) {
            lltable.set("status", "exceeded tolerated rotational rate");
            return;
        }

        lltable.set("status", "accepted!");
        updateVisionMeasurement(limelightName, mt1);
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

        // ensure reasonable minimums (x, y: 5cm, theta: 5°)
        stddevs.getData()[0] = Math.max(Centimeters.of(1).in(Meters), stddevs.getData()[0]);
        stddevs.getData()[1] = Math.max(Centimeters.of(1).in(Meters), stddevs.getData()[1]);
        stddevs.getData()[2] = Math.max(Degrees.of(5).in(Radians), stddevs.getData()[2]);

        NTable est = table.sub("estimate");
        NTable penalties = est.sub("penalties");

        // multiply by 1 + average distance to camera
        double distPenalty = 1 + estimate.distToCamera * 3;
        penalties.set("distance", distPenalty);
        // multiply by 1 + speed
        double speedPenalty = 1 + Math.hypot(
                drivetrain.getState().Speeds.vxMetersPerSecond,
                drivetrain.getState().Speeds.vyMetersPerSecond) * 5;
        penalties.set("speed", speedPenalty);
        // multiply by 1 + omega
        double omegaPenalty = 1 + Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) * 10;
        penalties.set("omega", omegaPenalty);
        // multiply by (1 + ambiguity)^2
        double totalAmbiguity = Arrays.stream(estimate.fiducials).mapToDouble(fiducial -> fiducial.ambiguity).sum();
        double ambiguityPenalty = 1 + Math.pow(1 * totalAmbiguity, 2);
        penalties.set("ambiguity", ambiguityPenalty);

        double singleTagPenalty = estimate.fiducials.length == 1 ? 10 : 1;
        penalties.set("one tag", singleTagPenalty);

        double totalPenalty = distPenalty * speedPenalty * omegaPenalty * ambiguityPenalty * singleTagPenalty;
        penalties.set("total", totalPenalty);

        stddevs = stddevs.times(totalPenalty);
        // if robot is rotating, use 99999 for theta
        if (drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble() > VisionConstants.MAX_ROTATION_SPEED
                .in(DegreesPerSecond)) {
            stddevs.getData()[2] = 99999;
        }

        est.sub("stddev").set("x", stddevs.getData()[0]);
        est.sub("stddev").set("y", stddevs.getData()[1]);
        est.sub("stddev").set("theta", stddevs.getData()[2]);

        est.set("x", estimate.pose2d.getX());
        est.set("y", estimate.pose2d.getY());
        est.set("theta", estimate.pose2d.getRotation().getDegrees());

        est.set("timestamp", estimate.timestamp);

        drivetrain.addVisionMeasurement(estimate.pose2d, estimate.timestamp, stddevs);
    }
}
