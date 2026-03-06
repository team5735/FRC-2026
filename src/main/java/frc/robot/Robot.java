// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveOnArc;
import frc.robot.commands.drivetrain.PIDToPose;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.constants.drivetrain.DevbotTunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FuelLauncherSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SpinDexSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.NTable;
import frc.robot.util.geometry.Arc;

public class Robot extends TimedRobot {
    public final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    public final CommandXboxController subsystemController = new CommandXboxController(
            Constants.SUBSYSTEM_CONTROLLER_PORT);

    public final CommandXboxController testController = new CommandXboxController(
            Constants.TEST_CONTROLLER_PORT);

    public final Telemetry logger = new Telemetry();

    public static final DrivetrainSubsystem drivetrain = switch (Constants.DRIVETRAIN_TYPE) {
        case COMPBOT -> CompbotTunerConstants.createDrivetrain();
        case DEVBOT -> DevbotTunerConstants.createDrivetrain();
    };

    public static final LimelightSubsystem limelights[] = {
            new LimelightSubsystem(drivetrain, "limelight-fone"),
            new LimelightSubsystem(drivetrain, "limelight-ftwo"),
    };

    public static final FuelLauncherSubsystem launcher = new FuelLauncherSubsystem();
    public static final TurretSubsystem turret = new TurretSubsystem(
            drivetrain::getEstimatedPosition);
    public static final ClimberSubsystem climber = new ClimberSubsystem();
    public static final SpinDexSubsystem spindex = new SpinDexSubsystem();

    public static final HoodSubsystem hood = new HoodSubsystem(turret::getMechanismPose,
            new Rectangle2d[] { FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_LEFT,
                    FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_RIGHT,
                    FieldConstants.redElement(FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_LEFT),
                    FieldConstants.redElement(FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_RIGHT),
            });

    public Robot() {
        NTable.root().set("mode", "full robot");
        NTable.root().set("scheduler", CommandScheduler.getInstance());

        configureBindings();
        setupAutoChooser();

        CommandScheduler.getInstance().schedule(
                PathfindingCommand.warmupCommand().withName("Warmup Pathfinding"));

        SignalLogger.enableAutoLogging(false);

        DriverStation.silenceJoystickConnectionWarning(true);

        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUToPigeon();
        }
    }

    private SendableChooser<Command> autoChooser;
    private Command storedAuto;

    private void setupAutoChooser() {
        Map<String, Command> commandsForAuto = new HashMap<>();

        // we disable the requirements so that the pid to pose command can control the
        // robot during the auto
        commandsForAuto.put("pid adjust", new PIDToPose(drivetrain, () -> {
            var pos = drivetrain.getEstimatedPosition();
            drivetrain.resetPose(limelights[1].new PoseEstimate().pose2d);
            return pos;
        }, "stay in place !", true));

        NamedCommands.registerCommands(commandsForAuto);

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Choose an Auto", autoChooser);
    }

    public static final Arc targetArc = new Arc(FieldConstants.BLUE_HUB_CENTER,
            Feet.of(7.5).in(Meters),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(270));

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        hood.exclusionZoneTrigger.onTrue(Commands.runOnce(() -> {
            SmartDashboard.putBoolean("in_exclusion_zone", true);
            // todo: add telemetry / debug / logging
            hood.exzSaveServoPosition();
            hood.setHoodPosition(0);
        }));
        hood.exclusionZoneTrigger.onFalse(Commands.runOnce(() -> {
            SmartDashboard.putBoolean("in_exclusion_zone", false);
            // todo: add telemetry / debug / logging
            double pos = hood.exzGetSavedServoPosition();
            hood.setServoPosition(pos);
        }));

        turret.setDefaultCommand(turret.holdRobotRel(Rotations.of(0.5)));
        turret.limitTrigger.onTrue(turret.zeroCommand()); // resets the turrets position when it engages the Hall-Effect
                                                          // sensor

        setupDriverBindings();
        setupOtherBindings();
    }

    private void setupDriverBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.joystickDriveCommand(
                        () -> driveController.getLeftX(),
                        () -> driveController.getLeftY(),
                        () -> driveController.getLeftTriggerAxis(),
                        () -> driveController.getRightTriggerAxis(),
                        () -> driveController.getHID().getBButton()));

        driveController.start().onTrue(drivetrain.runOnce(() -> {
            Pose2d pose = limelights[1].new PoseEstimate().pose2d;
            if (pose != null) {
                drivetrain.resetPose(pose);
            }
        }).withName("Resetting Pose"));

        driveController.x().whileTrue(new DriveOnArc(drivetrain, targetArc,
                () -> MathUtil.applyDeadband(driveController.getLeftX(), 0.1)));
        driveController.y()
                .onTrue(new PIDToPose(drivetrain,
                        () -> targetArc.getPoseFacingCenter(
                                targetArc.nearestPointOnArc(drivetrain.getEstimatedPosition().getTranslation())),
                        "drive to arc"));

        driveController.a().whileTrue(
                new DriveOnArc(drivetrain, targetArc, () -> MathUtil.applyDeadband(driveController.getLeftX(), 0.1)));
    }

    private void setupOtherBindings() {
        launcher.setDefaultCommand(launcher.getLaunchFuel(RPM.of(0), RPM.of(0)));

        testController.a().whileTrue(launcher.getFedLaunch(spindex, RPM.of(3000), RPM.of(24)));
        testController.b().whileTrue(launcher.getFedLaunch(spindex, RPM.of(1500), RPM.of(12)));
        testController.x().whileTrue(launcher.getFedLaunch(spindex, RPM.of(6000), RPM.of(48)));

        testController.povUp().onTrue(Commands.runOnce(() -> hood.setHoodPosition(0.7)));
        testController.povDown().onTrue(Commands.runOnce(() -> hood.setHoodPosition(0.3)));
        testController.povLeft().onTrue(turret.runOnce(() -> turret.remakePID()));

        testController.rightBumper().whileTrue(turret.testForward());
        testController.leftBumper().whileTrue(turret.testReverse());

        subsystemController.a().whileTrue(spindex.getStart());
    }

    @Override
    public void robotPeriodic() {
        for (LimelightSubsystem limelight : limelights) {
            limelight.handleVisionMeasurement();
        }

        CommandScheduler.getInstance().run();
        NTable.updateAllSendables();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUToPigeon();
        }
    }

    @Override
    public void autonomousInit() {
        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUMode(3);
        }

        Command auto = autoChooser.getSelected();
        if (auto == null) {
            DriverStation.reportWarning("null auto command retrieved!", false);
            return;
        }

        storedAuto = auto;
        CommandScheduler.getInstance().schedule(storedAuto);
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (storedAuto != null) {
            storedAuto.cancel();
        }

        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUMode(3);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
