// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NTDoubleSection;

public class VisionSubsystem extends SubsystemBase {
    DrivetrainSubsystem drivetrain;
    @SuppressWarnings("unused")
    private double driftEstimateTicks;

    public static final String LIMELIGHTS[] = { "limelight-left", "limelight-right" };

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

    private double lastTimestamp;

    private LimelightHelpers.PoseEstimate getMt2Estimate(String limelightName) {
        LimelightHelpers.SetRobotOrientation(limelightName, drivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0,
                0, 0, 0);
        doubles.set("drivetrainYaw", drivetrain.getPigeon2().getYaw().getValueAsDouble());
        doubles.set("drivetrainOmegaZ", drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());

        return LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    private boolean drivetrainIsNaNOrInf() {
        return Double.isNaN(drivetrain.getEstimatedPosition().getX())
                || Double.isInfinite(drivetrain.getEstimatedPosition().getX()) ||
                Double.isNaN(drivetrain.getEstimatedPosition().getY())
                || Double.isInfinite(drivetrain.getEstimatedPosition().getY());
    }

    public void maybeUpdateVisionMeasurement(String limelightName) {
        /*
         * algo thoughts:
         * get mt1 and mt2 estimates
         * get raw fiducials
         * reject vision measurement if any of these are true:
         *   mt1 and mt2 pose estimates differ significantly (> 20cm euclidean off from each other)
         *   mt1 pose estimate is more than 3cm off the ground
         *   mt1 pose estimate is off the field
         *   1 tag pose estimate and ambiguity > 0.2
         *   multi tag pose estimate and any tag ambiguity > 0.5
         *   robot is rotating (if we get angular velocity)
         *   
         *  use mt2 if !rejected and:
         *    pigeon was updated recently (<10s)
         *  use mt1 otherwise
         * 
         *  reset pigeon if:
         *    not updated recently (>5s) -- prevents spamming reset
         *    all pose estimate ambiguity < 0.2
         *    at least one tag is close (1.5m) TODO look at this constant in visionconstants
         *    yaw stddev < 5 degrees (I think it's in degrees)
         * 
         *   use stddevs:
         *     use the estimated stddevs for mt1 or mt2, depending on which type we're updating with
         *     multiply stddevs by avg tag dist to camera: stddev*(1+dist)
         *     multiply by robot velocity stddev*(1+mps)
         *     if robot is rotating, use 99999 for theta
         *     ensure proper minimums (x,y: 0.05, theta: 5 degrees, but in radians)
         */
        LimelightHelpers.PoseEstimate estimate;
        if (VisionConstants.IS_MT2) {
            estimate = getMt2Estimate(limelightName);
        } else {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }

        if (estimate == null) {
            doubles.set(limelightName + "_status", 1);
            return;
        }

        if (Math.abs(estimate.timestampSeconds - lastTimestamp) < Constants.TOLERANCE) {
            doubles.set(limelightName + "_status", 5);
            return;
        }
        lastTimestamp = estimate.timestampSeconds;

        if (estimate.tagCount == 1 && estimate.rawFiducials.length == 1) {
            var firstFiducial = estimate.rawFiducials[0];
            if (firstFiducial.ambiguity > .7) {
                doubles.set(limelightName + "_status", 2);
                return;
            } else if (limelightName == "limelight-right" // our Limelight 3
                    && firstFiducial.distToCamera > 3) {
                doubles.set(limelightName + "_status", 3);
                return;
            }
        } else if (estimate.tagCount == 0) {
            doubles.set(limelightName + "_status", 4);
            return;
        }

        updateVisionMeasurement(limelightName, estimate);
    }

    private double lastPigeonReset = 0;

    private void updateVisionMeasurement(String limelightName, LimelightHelpers.PoseEstimate estimate) {
        double timestamp = Timer.getFPGATimestamp();
        if (Arrays.stream(estimate.rawFiducials)
                .allMatch(tag -> tag.distToCamera < VisionConstants.RESET_PIGEON_DISTANCE.in(Meters))
                && Math.abs(lastPigeonReset - timestamp) > VisionConstants.RESET_PIGEON_INTERVAL.in(Seconds)) {
            drivetrain.getPigeon2().setYaw(estimate.pose.getRotation().getMeasure());
            lastPigeonReset = timestamp;
        }

        doubles.set(limelightName + "_status", 0);
        // if the estimate is more than 4 meters away from the current estimate, reset
        if (drivetrain.getEstimatedPosition().getTranslation()
                .getDistance(estimate.pose.getTranslation()) > 4
                || drivetrainIsNaNOrInf()) {
            drivetrain.resetPose(estimate.pose);
        }
        drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds,
                VecBuilder.fill(.7, .7, 9999999));
    }

    @Override
    public void periodic() {
        for (String limelight : LIMELIGHTS) {
            maybeUpdateVisionMeasurement(limelight);
        }
    }

    private void resetToMt1() {
        for (String limelight : LIMELIGHTS) {
            if (LimelightHelpers.getTV(limelight)) {
                drivetrain.resetPose(LimelightHelpers.getBotPose2d(limelight));
                System.out.println("Robot pose set to mt1 report from " + limelight);
                break;
            }
        }
    }

    public Command getWaitForMt1() {
        return Commands.idle() // wait
                .until(() -> { // until any limelight sees a tag
                    return Arrays.stream(LIMELIGHTS).anyMatch(limelight -> LimelightHelpers.getTV(limelight));
                })
                .andThen(Commands.run(() -> resetToMt1(), drivetrain).withTimeout(.1)) // reset pose
        ;
    }
}
