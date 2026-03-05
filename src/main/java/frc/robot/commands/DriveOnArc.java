package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.NTable;
import frc.robot.util.TunablePIDController;
import frc.robot.util.geometry.Arc;

public class DriveOnArc extends Command {
    private DrivetrainSubsystem drivetrain;
    private Arc arc;
    // expected to be between -1 and 1
    private Supplier<Double> control;

    private TunablePIDController pidTheta = new TunablePIDController("drive on arc theta");

    private NTable table = NTable.root("drive on arc");

    public DriveOnArc(DrivetrainSubsystem drivetrain, Arc arc, Supplier<Double> movement) {
        this.drivetrain = drivetrain;
        this.arc = arc;
        this.control = movement;
        this.pidTheta.ensureTolerance(Degrees.of(15).in(Radians));
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d target = arc.getPoseFacingCenter(
                arc.nearestPointOnArc(drivetrain.getEstimatedPosition().getTranslation()));

        this.pidTheta.setup(target.getRotation().getRadians());
        this.pidTheta.getController().enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        double control = this.control.get();
        double speed = DrivetrainSubsystem.CONSTANTS.getDefaultSpeed().in(MetersPerSecond);

        Pose2d robotPose = drivetrain.getEstimatedPosition();
        Pose2d nearestPose = arc.getPoseFacingCenter(
                arc.nearestPointOnArc(robotPose.getTranslation()));
        Translation2d tangentialMovement = new Translation2d(2 * control * speed,
                nearestPose.getRotation().plus(Rotation2d.kCCW_90deg));

        Translation2d radialWish = nearestPose.getTranslation().minus(robotPose.getTranslation()).times(speed);
        this.table.set("radial wish", radialWish);
        if (radialWish.getNorm() < Centimeters.of(7).in(Meters) && control < 0.3) {
            radialWish = new Translation2d();
        }

        double omega = this.pidTheta.calculate(robotPose.getRotation().getRadians(),
                nearestPose.getRotation().getRadians());
        if (this.pidTheta.atSetpoint() && control == 0) {
            omega = 0;
        }

        this.table.set("tangential movement", tangentialMovement);
        Translation2d horizontal = tangentialMovement.plus(radialWish);
        this.table.set("horizontal", horizontal);
        drivetrain.pidDrive(horizontal, omega);
    }
}
