package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.NTable;
import frc.robot.util.TunablePIDController;

/**
 * This is a relatively generic command whose purpose is to use
 * TunablePIDControllers to drive the robot to a specified pose.
 *
 * <p>
 * The pose is retrieved every time the command is initialized, which is also
 * when the constants for the PID controllers are retrieved from NetworkTables
 * to facilitate runtime tuning of the PIDs.
 */
public class PIDToPose extends Command {
    private Supplier<Pose2d> poseSupplier;
    private Pose2d targetPose;

    private TunablePIDController pidX;
    private TunablePIDController pidY;
    private TunablePIDController pidTheta;

    private DrivetrainSubsystem drivetrain;

    public PIDToPose(DrivetrainSubsystem drivetrain, Supplier<Pose2d> poseSupplier, String name) {
        this(drivetrain, poseSupplier, name, false);
    }

    public PIDToPose(DrivetrainSubsystem drivetrain, Supplier<Pose2d> poseSupplier, String name,
            boolean skipRequirement) {
        this.drivetrain = drivetrain;
        this.poseSupplier = poseSupplier;
        NTable table = NTable.root("pid to pose").sub(name);
        this.pidX = new TunablePIDController(table, "x");
        this.pidY = new TunablePIDController(table, "y");
        this.pidTheta = new TunablePIDController(table, "theta");
        this.pidTheta.setContinuous(-Math.PI, Math.PI);

        this.pidX.ensureTolerance(Centimeters.of(2).in(Meters));
        this.pidY.ensureTolerance(Centimeters.of(2).in(Meters));
        this.pidTheta.ensureTolerance(Degrees.of(2).in(Radians));

        this.pidX.ensureP(1);
        this.pidY.ensureP(1);
        this.pidTheta.ensureP(1);

        if (!skipRequirement) {
            addRequirements(drivetrain);
        }

        setName("PIDToPose: " + name);
    }

    @Override
    public void initialize() {
        this.targetPose = this.poseSupplier.get();
        this.pidX.setup(targetPose.getX());
        this.pidY.setup(targetPose.getY());
        this.pidTheta.setup(targetPose.getRotation().getRadians());
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
