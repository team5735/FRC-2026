package frc.pathplanner;

import org.junit.jupiter.api.Test;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.drivetrain.CompbotConstants;
import frc.robot.util.PathPlannerHelpers;

import edu.wpi.first.math.controller.PIDController;


public class PathPlannerTest {
    @Test
    void linePathTest() throws Exception {
        CompbotConstants bot = new CompbotConstants();

        PathPlannerPath path = PathPlannerHelpers.createPathBetween(
                new Translation2d(0, 0),
                new Translation2d(1, 1),
                Rotation2d.fromDegrees(-45),
                new PathConstraints(3.0, 2.0, 360, 540),
                true);

        PathPlannerTrajectory trajectory = path.generateTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(0),
                bot.getConfig());

        // Sample points for plotting
        System.out.printf("x_meters, y_meters, rotation_degrees\n");
        for (var s : trajectory.getStates()) {
            Translation2d pos = s.pose.getTranslation();
            System.out.printf("%.2f, %.2f, %.2f\n",
                    pos.getX(),
                    pos.getY(),
                    s.pose.getRotation().getDegrees());
        }
    }

    @Test
    void continuousPIDTest() throws Exception {
        System.out.printf("here!\n");
        PIDController controller;

        controller = new PIDController(1,0,0);
        controller.enableContinuousInput(0, 10);

        controller.setSetpoint(0);
        controller.setTolerance(0);

        System.out.printf("setpoint: 0\n");
        for (int x=-10; x<20; x++){
            double e = controller.calculate(x);
            System.out.printf("  measurement: %d, error: %f\n", x, e);
        }

        controller.setSetpoint(15);
        System.out.printf("setpoint: 15 (5)\n");
        for (int x=-10; x<20; x++){
            double e = controller.calculate(x);
            System.out.printf("  measurement: %d, error: %f\n", x, e);
        }

    }    
}
