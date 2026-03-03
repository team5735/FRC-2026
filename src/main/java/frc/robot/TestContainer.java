package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.HoodSubsystem;

public class TestContainer {
    private static final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

//     private static final HoodSubsystem hood = new HoodSubsystem();

    public static void configureBindings() {
        // driveController.a().onTrue(hood.runOnce(() -> hood
        //         .setPosition(SmartDashboard.getNumber("End Revolution Posiiton", Constants.END_REVOLUTION_POSITION))));
        // driveController.a().onFalse(hood.runOnce(() -> hood.setPosition(
        //         SmartDashboard.getNumber("Start Revolution Posiiton", Constants.START_REVOLUTION_POSITION))));
    }

    public static final HoodSubsystem hood = RobotContainer.hood;
    public static double hoodTuningServoPosition = 0.5;
    public static double hoodTuningServoPositionDx = 0.05;
    public static void configureHoodTuningBindings(){
        driveController.x().onTrue(Commands.runOnce(() -> {
            hoodTuningServoPosition += hoodTuningServoPositionDx;
            hoodTuningServoPosition = MathUtil.clamp(hoodTuningServoPosition, 0.0, 1.0);
            hood.setServoPosition(hoodTuningServoPosition);
            SmartDashboard.putNumber("hood/tuning_servo_position", hoodTuningServoPosition);
        }));
        driveController.a().onTrue(Commands.runOnce(() -> {
            hoodTuningServoPosition -= hoodTuningServoPositionDx;
            hoodTuningServoPosition = MathUtil.clamp(hoodTuningServoPosition, 0.0, 1.0);
            hood.setServoPosition(hoodTuningServoPosition);
            SmartDashboard.putNumber("hood/tuning_servo_position", hoodTuningServoPosition);
        }));
    }
}
