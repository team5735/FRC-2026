// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.constants.drivetrain.DevbotTunerConstants;
import frc.robot.constants.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
    private final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    private final CommandXboxController testController = new CommandXboxController(
            Constants.TEST_CONTROLLER_PORT);

    private final Telemetry logger = new Telemetry();

    public static final DrivetrainSubsystem drivetrain;

    static {
        switch (Constants.DRIVETRAIN_TYPE) {
            case COMPBOT:
                drivetrain = CompbotTunerConstants.createDrivetrain();
                break;
            case DEVBOT:
                drivetrain = DevbotTunerConstants.createDrivetrain();
                break;
            default:
                throw new RuntimeException("Unknown drivetrain type");
        }
    }

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        drivetrain.setDefaultCommand(drivetrain.joystickDriveCommand(
                () -> driveController.getLeftX(),
                () -> driveController.getLeftY(),
                () -> driveController.getLeftTriggerAxis(),
                () -> driveController.getRightTriggerAxis(),
                () -> driveController.getHID().getBButton()));

        driveController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        testController.a().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        testController.b().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        testController.x().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        testController.y().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        testController.rightBumper().whileTrue(drivetrain.applyRequest(
                () -> {
                    double x = testController.getRightX();
                    double y = testController.getRightY();
                    Angle theta = Radians.of(Math.atan(y / x));
                    return new SwerveRequest.PointWheelsAt().withModuleDirection(new Rotation2d(theta));
                }));

    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
