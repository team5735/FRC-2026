package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        this.pidX.setup(targetPose.getX(), 0.02);
        this.pidY.setup(targetPose.getY(), 0.02);
        this.pidTheta.setup(targetPose.getRotation().getRadians(), 0.02);
        this.pidTheta.getController().enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        Translation2d deltaTrans = this.targetPose.getTranslation()
                .minus(drivetrain.getEstimatedPosition().getTranslation());
        Rotation2d deltaRot = this.targetPose.getRotation()
                .minus(drivetrain.getEstimatedPosition().getRotation());
        double vx = this.pidX.calculate(deltaTrans.getX());
        double vy = this.pidY.calculate(deltaTrans.getY());
        double omega = this.pidTheta.calculate(deltaRot.getRadians());
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
