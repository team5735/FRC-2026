// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.constants.FieldConstants;
import frc.robot.util.NTable;
import frc.robot.util.Timer;

public class LimelightSubsystem extends SubsystemBase {
    /*
     * In the Limelight web ui, under configuration, april tag
     * Area (% of image) min/max sliders:
     *  0.1506 -- 1.0
     * 
     * exposure: 240 (2.40ms)
     * gain: 3
     * 
     * MegaTag Field-Space Localization Setup
     * fone:
     *  forward: -0.263525
     *    right: -0.37465
     *       up: 0.42545
     *     roll: 180
     *    pitch: 0
     *      yaw: 90
     * ftwo:
     *  forward: -0.263525
     *    right: 0.37465
     *       up: 0.2445
     *     roll: 0
     *    pitch: 0
     *      yaw: -90
     * 
     * 
     * 
     * 
     */

    private final DrivetrainSubsystem drivetrain;
    private final String limelightName;
    private final NTable table;

    private final NTable lltable;

    private final NTable lltLimits;
    private final NTable lltChecks;
    private final NTable lltMeasurements;
    private final NTable lltEstimate;
    private final NTable lltEstimateMeasurements;
    private final NTable lltEstimatePenalties;
    private final NTable lltEstimateCoefficients;


    public LimelightSubsystem(
            DrivetrainSubsystem drivetrain,
            String limelightName) {
        super();

        this.limelightName = limelightName;
        this.drivetrain = drivetrain;
        this.table = NTable.root("vision").sub(limelightName);
        this.lltLimits = table.sub("limits");
        this.lltChecks = table.sub("checks");
        this.lltMeasurements = table.sub("measurements");
        this.lltEstimate = table.sub("estimate");
        this.lltEstimateMeasurements = table.sub("estimate").sub("measurements");
        this.lltEstimatePenalties = table.sub("estimate").sub("penalties");
        this.lltEstimateCoefficients = table.sub("estimate").sub("coefficients");

        NTable.root("vision").ensure("enabled", true);

        lltLimits.ensure("distance from ground", Centimeters.of(25).in(Meters));
        lltLimits.ensure("angular velocity", DegreesPerSecond.of(10).in(RadiansPerSecond));
        lltLimits.ensure("single tag ambiguity", 0.2);
        lltLimits.ensure("multi tag ambiguity", 0.5);
        lltLimits.ensure("(stddevs) angular velocity", DegreesPerSecond.of(1).in(RadiansPerSecond));
        lltLimits.ensure("drivetrain distance from estimate", 1);

        lltable = NTable.root(limelightName);
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

        public PoseEstimate() {
            double[] stddevs = lltable.get("stddevs", new double[0]);
            double[] array = lltable.get("botpose_wpiblue", new double[0]);
            double timestamp = lltable.getEntry("botpose_wpiblue").getLastChange();

            if (array.length == 0 || stddevs.length == 0) {
                return;
            }
            int nFiducials = (int) array[7];
            if (nFiducials == 0) {
                return;
            }
            Translation3d translation = new Translation3d(
                    Meters.of(array[0]),
                    Meters.of(array[1]),
                    Meters.of(array[2]));
            Rotation3d rotation = new Rotation3d(
                    Degrees.of(array[3]),
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
            this.timestamp = (timestamp / 1e6) - (latency / 1e3);
            this.fiducials = fiducials;
            this.distToCamera = array[9];

            this.stddevs = new Pose3d(
                    new Translation3d(
                            Meters.of(stddevs[0]),
                            Meters.of(stddevs[1]),
                            Meters.of(stddevs[2])),
                    new Rotation3d(
                            Degrees.of(stddevs[3]),
                            Degrees.of(stddevs[4]),
                            Degrees.of(stddevs[5])));
        }
    }

    public void setIMUMode(int mode) {
        this.lltable.set("imumode_set", mode);
    }

    public void setIMUToPigeon() {
        setIMUMode(1);

        Pose2d pose = drivetrain.getEstimatedPosition();
        // format is yaw, yawRate, pitch, pitchRate, roll, rollRate
        this.lltable.set(
                "robot_orientation_set",
                new double[] { pose.getRotation().getDegrees(), 0, 0, 0, 0, 0 });
    }

    private boolean check(double measurement, String name) {
        this.lltMeasurements.set(name, measurement);
        boolean ok = measurement <= this.lltLimits.get(name, 0.0);
        this.lltChecks.set(name, ok);
        return ok;
    }

    public Pose2d getPoseEstimate() {
        return new PoseEstimate().pose2d;
    }

    @Override
    public void periodic() {
        var _T = new Timer("LimelightSubsystem."+limelightName);
        handleVisionMeasurement();
        _T.toc();
    }

    public void handleVisionMeasurement() {
        if (!NTable.root("vision").get("enabled", true)) {
            this.lltChecks.set("enabled in network tables", false);
            return;
        }
        this.lltChecks.set("enabled in network tables", true);

        PoseEstimate estimate = new PoseEstimate();
        if (estimate.pose2d == null) {
            this.lltChecks.set("could retrieve pose estimate", false);
            return;
        }
        this.lltChecks.set("could retrieve pose estimate", true);

        Telemetry.field.getObject(limelightName).setPose(estimate.pose2d);

        boolean accepted = true;

        accepted = check(
                estimate.pose3d.getMeasureZ().in(Meters),
                "distance from ground") &&
                accepted;

        // pose estimate is off the field
        double conservativeRobotRadius = (Math.max(
                drivetrain.constants.getRobotTotalLength().in(Meters),
                drivetrain.constants.getRobotTotalWidth().in(Meters))
                * Math.sqrt(2)) / 2;

        if (estimate.pose2d.getTranslation().getX() < conservativeRobotRadius
                || estimate.pose2d.getTranslation()
                        .getX() > (FieldConstants.FIELD_LENGTH_X.in(Meters) - conservativeRobotRadius)
                || estimate.pose2d.getTranslation().getY() < conservativeRobotRadius || estimate.pose2d.getTranslation()
                        .getY() > (FieldConstants.FIELD_LENGTH_Y.in(Meters) - conservativeRobotRadius)) {
            this.lltChecks.set("in field", false);
            accepted = false;
        }
        this.lltChecks.set("in field", true);

        double[] ambiguities = Arrays.stream(estimate.fiducials)
                .mapToDouble(tag -> tag.ambiguity)
                .toArray();
        if (ambiguities.length == 1) {
            accepted = check(
                    ambiguities[0],
                    "single tag ambiguity") && accepted;
        } else if (ambiguities.length > 1) {
            accepted = check(
                    Arrays.stream(ambiguities).max().getAsDouble(),
                    "multi tag ambiguity") && accepted;
        }

        accepted = check(
                drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
                "angular velocity")
                && accepted;

        if (!accepted) {
            return;
        }

        updateVisionMeasurement(estimate);
    }

    private boolean drivetrainIsNaNOrInf() {
        return (Double.isNaN(drivetrain.getEstimatedPosition().getX()) ||
                Double.isInfinite(drivetrain.getEstimatedPosition().getX()) ||
                Double.isNaN(drivetrain.getEstimatedPosition().getY()) ||
                Double.isInfinite(drivetrain.getEstimatedPosition().getY()));
    }

    private double penalize(double measurement, String penaltyName) {
        lltEstimateMeasurements.set(penaltyName, measurement);
        double penalty = 1 + measurement * this.lltEstimateCoefficients.get(penaltyName, 1.0);
        this.lltEstimatePenalties.set(penaltyName, penalty);
        return penalty;
    }

    private void updateVisionMeasurement(PoseEstimate estimate) {
        // completely reset the pose estimator if it's not having a good time
        if (drivetrainIsNaNOrInf()) {
            System.out.println(
                    "resetting drivetrain pose estimator due to nan or inf");
            drivetrain.resetPose(estimate.pose2d);
        }

        lltEstimateCoefficients.ensure("distance", 3);
        lltEstimateCoefficients.ensure("speed", 5);
        lltEstimateCoefficients.ensure("omega", 10);
        lltEstimateCoefficients.ensure("ambiguity", 1);
        lltEstimateCoefficients.ensure("single tag", 3);

        Vector<N3> stddevs = VecBuilder.fill(
                estimate.stddevs.getX(),
                estimate.stddevs.getY(),
                estimate.stddevs.getRotation().getMeasureZ().in(Radians));

        // ensure reasonable minimums (x, y: 5cm, theta: 5°)
        stddevs.getData()[0] = Math.max(
                Centimeters.of(1).in(Meters),
                stddevs.getData()[0]);
        stddevs.getData()[1] = Math.max(
                Centimeters.of(1).in(Meters),
                stddevs.getData()[1]);
        stddevs.getData()[2] = Math.max(
                Degrees.of(5).in(Radians),
                stddevs.getData()[2]);

        double distPenalty = penalize(
                estimate.distToCamera,
                "distance");

        double speedPenalty = penalize(
                Math.hypot(
                        drivetrain.getState().Speeds.vxMetersPerSecond,
                        drivetrain.getState().Speeds.vyMetersPerSecond) * 5,
                "speed");

        double omegaPenalty = penalize(Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond), "omega");

        double ambiguityPenalty = penalize(
                Arrays.stream(estimate.fiducials)
                        .mapToDouble(fiducial -> fiducial.ambiguity)
                        .sum(),
                "ambiguity");
        ambiguityPenalty *= ambiguityPenalty;

        double singleTagPenalty = penalize(
                estimate.fiducials.length == 1 ? 3.33 : 0,
                "single tag");

        double totalPenalty = distPenalty *
                speedPenalty *
                omegaPenalty *
                ambiguityPenalty *
                singleTagPenalty;
        this.lltEstimatePenalties.set("total", totalPenalty);

        stddevs = stddevs.times(totalPenalty);
        if (!check(
                drivetrain
                        .getPigeon2()
                        .getAngularVelocityZWorld()
                        .getValueAsDouble(),
                "(stddevs) angular velocity")) {
            stddevs.getData()[2] = 99999;
        }

        this.lltEstimate.set(
                "stddevs",
                new Pose2d(
                        stddevs.getData()[0],
                        stddevs.getData()[1],
                        Rotation2d.fromRadians(stddevs.getData()[2])));
        this.lltEstimate.set("pose", estimate.pose2d);

        this.lltEstimate.set("timestamp", estimate.timestamp);

        drivetrain.addVisionMeasurement(
                estimate.pose2d,
                estimate.timestamp,
                stddevs);
    }
}
