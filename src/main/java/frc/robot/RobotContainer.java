// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    public static final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    public static final CommandXboxController testController = new CommandXboxController(
            Constants.TEST_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;
    private IntakeSubsystem intake;

    public RobotContainer() {
        Map<String, Command> commandsForAuto = new HashMap<>();

        NamedCommands.registerCommands(commandsForAuto);

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Choose an Auto", autoChooser);
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        DriverStation.silenceJoystickConnectionWarning(true);
        configureBindings();
    }



    private void configureBindings() {
        driveController.a().whileTrue(intake.getIntakeRollerCommand());
        driveController.b().whileTrue(intake.getIntakeReverseCommand());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
