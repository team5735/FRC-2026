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
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.commands.MoveCommand;

public class RobotContainer {
    public HoodSubsystem hood = new HoodSubsystem();
    public static final CommandXboxController servoController = new CommandXboxController(Constants.SERVO_CONTROLLER_PORT);

    //drivetrain
    public static final CommandXboxController driveController = new CommandXboxController(
        Constants.DRIVE_CONTROLLER_PORT);
        private final SendableChooser<Command> autoChooser;
    //end drivetrain

    

    

    public RobotContainer() {
        configureBindings();

        //drivetrain
        Map<String, Command> commandsForAuto = new HashMap<>();

        NamedCommands.registerCommands(commandsForAuto);

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Choose an Auto", autoChooser);
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        DriverStation.silenceJoystickConnectionWarning(true);
        //end drivetrain
    }

    private void configureBindings() {
        servoController.a().whileTrue(new MoveCommand(hood, 0.2));
        servoController.b().whileTrue(new MoveCommand(hood, 0.8));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
