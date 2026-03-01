// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveOnArc;
import frc.robot.commands.drivetrain.PIDToPose;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.constants.drivetrain.DevbotTunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.geometry.Arc;

public class RobotContainer {
    public static final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    public static final CommandXboxController turretController = new CommandXboxController(
            Constants.TURRET_CONTROLLER_PORT);

    public static final CommandXboxController testController = new CommandXboxController(
            Constants.TEST_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry();

    public static final DrivetrainSubsystem drivetrain = switch (Constants.DRIVETRAIN_TYPE) {
            case COMPBOT -> CompbotTunerConstants.createDrivetrain();
            case DEVBOT -> DevbotTunerConstants.createDrivetrain();
        };

    public static final LimelightSubsystem limelights[] = { new LimelightSubsystem(drivetrain, "limelight-left") };

    public RobotContainer() {
        Map<String, Command> commandsForAuto = new HashMap<>();

        NamedCommands.registerCommands(commandsForAuto);

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Choose an Auto", autoChooser);
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand().withName("Warmup Pathfinding"));

        SignalLogger.enableAutoLogging(false);

        DriverStation.silenceJoystickConnectionWarning(true);
        configureBindings();
    }

    public static Arc targetArc = new Arc(FieldConstants.BLUE_HUB_CENTER,
            Feet.of(7.5).in(Meters), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(270));

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        drivetrain.setDefaultCommand(drivetrain.joystickDriveCommand(
                () -> driveController.getLeftX(),
                () -> driveController.getLeftY(),
                () -> driveController.getLeftTriggerAxis(),
                () -> driveController.getRightTriggerAxis(),
                () -> driveController.getHID().getBButton()));

        driveController.a().onTrue(drivetrain
                .runOnce(() -> drivetrain.resetPose(limelights[0].new PoseEstimate().pose2d))
                .withName("Resetting Pose"));

        driveController.x().whileTrue(
                new DriveOnArc(drivetrain, targetArc, () -> MathUtil.applyDeadband(driveController.getLeftX(), 0.1)));
        driveController.y()
                .onTrue(new PIDToPose(drivetrain,
                        () -> targetArc.getPoseFacingCenter(
                                targetArc.nearestPointOnArc(
                                        drivetrain.getEstimatedPosition().getTranslation())),
                        "drive to arc"));

        testController.a().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        testController.b().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        testController.x().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        testController.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        testController.rightBumper().whileTrue(drivetrain.applyRequest(
                () -> {
                    double x = testController.getRightX();
                    double y = testController.getRightY();
                    return new SwerveRequest.PointWheelsAt().withModuleDirection(new Rotation2d(x, y));
                }));
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
