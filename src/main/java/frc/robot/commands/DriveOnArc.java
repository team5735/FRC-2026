package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.TunablePIDController;
import frc.robot.util.geometry.Arc;

public class DriveOnArc extends Command {
    private DrivetrainSubsystem drivetrain;
    private Arc arc;
    private Supplier<Double> movement;

    private TunablePIDController pidX = new TunablePIDController("drive on arc x");
    private TunablePIDController pidY = new TunablePIDController("drive on arc y");
    private TunablePIDController pidTheta = new TunablePIDController("drive on arc theta");

    public DriveOnArc(DrivetrainSubsystem drivetrain, Arc arc, Supplier<Double> movement) {
        this.drivetrain = drivetrain;
        this.arc = arc;
        this.movement = movement;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d target = arc.getPoseFacingCenter(
                arc.nearestPointOnArc(drivetrain.getEstimatedPosition().getTranslation()));

        this.pidX.setup(target.getX(), Centimeters.of(2).in(Meters));
        this.pidY.setup(target.getY(), Centimeters.of(2).in(Meters));
        this.pidTheta.setup(target.getRotation().getRadians(), Degrees.of(2).in(Radians));
        this.pidTheta.getController().enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getEstimatedPosition();
        Pose2d nearestPose = arc.getPoseFacingCenter(
                arc.nearestPointOnArc(robotPose.getTranslation()));
        Translation2d tangentialMovement = new Translation2d(2 * this.movement.get(),
                nearestPose.getRotation().plus(Rotation2d.k_CW_90deg));

        double vx = this.pidX.calculate(robotPose.getX(), nearestPose.getX());
        double vy = this.pidY.calculate(robotPose.getY(), nearestPose.getY());
        double omega = this.pidTheta.calculate(robotPose.getRotation().getRadians(),
                nearestPose.getRotation().getRadians());

        drivetrain.pidDrive(vx + tangentialMovement.getX(), vy + tangentialMovement.getY(), omega);
    }
}
