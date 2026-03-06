package frc.robot.commands.drivetrain;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This is a relatively generic command that uses PathPlannerLib's on-the-fly
 * pathing capabilities to create a path and follow it.
 *
 * <p> This class utilizes a supplier so that the target pose can be set
 * multiple times during program execution, instead of just at robot startup.
 *
 * <p>
 * The poses are always assumed to be in blue-alliance space, and PathPlanner
 * is requested to not flip the path this command generates in order to enforce
 * consistency.
 */
public class PathPlannerToPose extends Command {
    private DrivetrainSubsystem drivetrain;
    private Supplier<Pose2d> poseSupplier;

    private Pose2d targetPose;

    public PathPlannerToPose(DrivetrainSubsystem drivetrain, Supplier<Pose2d> poseSupplier) {
        this.drivetrain = drivetrain;
        this.poseSupplier = poseSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        PathConstraints constraints = drivetrain.constants.getPathFollowConstraints();

        this.targetPose = this.poseSupplier.get();
        Pose2d currentRobotPos = drivetrain.getEstimatedPosition();
        Rotation2d botToTarget = targetPose.getTranslation().minus(currentRobotPos.getTranslation()).getAngle();
        Pose2d firstPose = new Pose2d(currentRobotPos.getTranslation(), botToTarget);
        Pose2d lastPose = new Pose2d(this.targetPose.getTranslation(), botToTarget);
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(firstPose, lastPose);
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints,
                new IdealStartingState(0, currentRobotPos.getRotation()),
                new GoalEndState(0, this.targetPose.getRotation()));
        path.preventFlipping = true;

        // replace this command with the pathfindThenFollowPath command
        CommandScheduler.getInstance().schedule(AutoBuilder.pathfindThenFollowPath(path, constraints));
        this.cancel();
    }

    // as a failsafe, ensure we're not left hogging the drivetrain
    @Override
    public boolean isFinished() {
        return true;
    }
}
