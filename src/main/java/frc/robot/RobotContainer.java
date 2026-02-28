// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.drivetrain.PIDToPose;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.constants.drivetrain.DevbotTunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FuelLauncherSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.geometry.Rectangle;

public class RobotContainer {
    public static final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    public static final CommandXboxController testController = new CommandXboxController(
            Constants.TEST_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

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

    public static final FuelLauncherSubsystem launcher = new FuelLauncherSubsystem();
    public static final TurretSubsystem turret = new TurretSubsystem(drivetrain::getEstimatedPosition);
    public static final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    

    
    public static final HoodSubsystem hood = new HoodSubsystem(turret::getMechanismPose, 
    new Rectangle[]{FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_LEFT,
         FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_RIGHT,
         FieldConstants.redElement(FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_LEFT),
         FieldConstants.redElement(FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_RIGHT)});

    public RobotContainer() {
        configureBindings();

        Map<String, Command> commandsForAuto = new HashMap<>();

        NamedCommands.registerCommands(commandsForAuto);

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Choose an Auto", autoChooser);
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        SignalLogger.enableAutoLogging(false);

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private Rotation2d getRightStickAsRotation() {
        double x = driveController.getRightX();
        double y = driveController.getRightY();
        if (x == 0 && y == 0) {
            return Rotation2d.kZero;
        }
        return new Rotation2d(x, y);
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
        driveController.x().onTrue(turret.holdFieldRelative(Rotations.of(0)));
        driveController.y()
                .onTrue(new PIDToPose(drivetrain,
                        () -> drivetrain.getEstimatedPosition()
                                .plus(new Transform2d(
                                        new Translation2d(1, getRightStickAsRotation()),
                                        Rotation2d.kZero)),
                        "straight line"));

        launcher.setDefaultCommand(launcher.getLaunchFuel(RPM.of(0), RPM.of(0)));

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

        testController.a().whileTrue(launcher.getLaunchFuel(RPM.of(3000), RPM.of(24)));
        testController.b().whileTrue(launcher.getLaunchFuel(RPM.of(1500), RPM.of(12)));
        testController.x().whileTrue(launcher.getLaunchFuel(RPM.of(6000), RPM.of(48)));

        testController.y().whileTrue(turret.trackRobotRel(() -> {
            double x = -testController.getRightY();
            double y = -testController.getRightX();
            Angle theta = new Rotation2d(x, y).getMeasure();
            SmartDashboard.putNumber("testTheta", theta.in(Rotations));
            return theta;
        }));

        testController.povUp().onTrue(turret.runOnce(() -> turret.remakePID()));
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
