package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.TunablePIDController;

public class PIDToPose extends Command {
    private Supplier<Pose2d> poseSupplier;
    private Pose2d targetPose;

    private TunablePIDController pidX;
    private TunablePIDController pidY;
    private TunablePIDController pidTheta;

    private DrivetrainSubsystem drivetrain;

    public PIDToPose(DrivetrainSubsystem drivetrain, Supplier<Pose2d> poseSupplier, String name) {
        this.drivetrain = drivetrain;
        this.poseSupplier = poseSupplier;
        this.pidX = new TunablePIDController(name + " drive to pose x");
        this.pidY = new TunablePIDController(name + " drive to pose y");
        this.pidTheta = new TunablePIDController(name + " drive to pose theta");

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.targetPose = this.poseSupplier.get();
        this.pidX.setup(targetPose.getX(), Centimeters.of(2).in(Meters));
        this.pidY.setup(targetPose.getY(), Centimeters.of(2).in(Meters));
        this.pidTheta.setup(targetPose.getRotation().getRadians(), Degrees.of(2).in(Radians));
        this.pidTheta.getController().enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getEstimatedPosition();
        double vx = this.pidX.calculate(robotPose.getX());
        double vy = this.pidY.calculate(robotPose.getY());
        double omega = this.pidTheta.calculate(robotPose.getRotation().getRadians());
        drivetrain.pidDrive(vx, vy, omega);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drivetrain.brakeRequest);
    }

    @Override
    public boolean isFinished() {
        return this.pidX.atSetpoint() && this.pidY.atSetpoint() && this.pidTheta.atSetpoint();
    }
}
