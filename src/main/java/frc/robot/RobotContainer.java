// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.constants.drivetrain.DevbotTunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.SpinDexSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    public static final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    public static final CommandXboxController testController = new CommandXboxController(
            Constants.TEST_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry();

    public static final DrivetrainSubsystem drivetrain;
    public static final TurretSubsystem turret = new TurretSubsystem();

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

    public static final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    public static final SpinDexSubsystem spindex = new SpinDexSubsystem();

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
        drivetrain.registerTelemetry(logger::telemeterize);

        drivetrain.setDefaultCommand(drivetrain.joystickDriveCommand(
                () -> driveController.getLeftX(),
                () -> driveController.getLeftY(),
                () -> driveController.getLeftTriggerAxis(),
                () -> driveController.getRightTriggerAxis(),
                () -> driveController.getHID().getBButton()));

        turret.setDefaultCommand(turret.holdRobotRel(Rotations.of(0.5)));

        driveController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driveController.b().onTrue(spindex.getStart());
        driveController.x().onTrue(turret.holdFieldRelative(Rotations.of(0),
                () -> drivetrain.getEstimatedPosition().getRotation().getMeasure()));
        driveController.y()
                .onTrue(turret.trackFieldPos(FieldConstants.BLUE_HUB_CENTER, drivetrain::getEstimatedPosition));

        testController.x().onTrue(turret.holdRobotRel(Rotations.of(0.5)));
        testController.y().whileTrue(turret.trackRobotRel(() -> {
            double x = -testController.getRightY();
            double y = -testController.getRightX();
            Angle theta = new Rotation2d(x, y).getMeasure();
            SmartDashboard.putNumber("testTheta", theta.in(Rotations));
            return theta;
        }));

        testController.povDown().whileTrue(turret.sysId());
        testController.rightBumper().whileTrue(turret.testForward());
        testController.leftBumper().whileTrue(turret.testReverse());
    }

    public Command getAutonomousCommand() {
        Command auto = autoChooser.getSelected();
        if (auto == null) {
            System.out.println("auto is null");
            return drivetrain.brakeCommand();
        }

        return auto;
    }

}
