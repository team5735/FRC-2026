// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.constants.drivetrain.DevbotTunerConstants;
import frc.robot.constants.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    public static final DrivetrainSubsystem drivetrain;

    static {
        switch (DrivetrainConstants.DRIVETRAIN_TYPE) {
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
        drivetrain.setDefaultCommand(drivetrain.joystickDriveCommand(
                () -> driverController.getLeftX(),
                () -> driverController.getLeftY(),
                () -> driverController.getLeftTriggerAxis(),
                () -> driverController.getRightTriggerAxis(),
                () -> driverController.getHID().getBButton()));

        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
