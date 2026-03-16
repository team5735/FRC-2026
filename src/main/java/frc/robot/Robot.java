// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveOnArc;
import frc.robot.commands.LaunchCalculator;
import frc.robot.commands.LaunchCalculator.LaunchGoal;
import frc.robot.commands.drivetrain.PIDToPose;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SpinDexSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.MatchState;
import frc.robot.util.NTable;
import frc.robot.util.geometry.Arc;

public class Robot extends TimedRobot {
    public final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    public final CommandXboxController subsystemController = new CommandXboxController(
            Constants.SUBSYSTEM_CONTROLLER_PORT);

    public final CommandXboxController testController = new CommandXboxController(
            Constants.TEST_CONTROLLER_PORT);

    public Arc targetArc;

    public final DrivetrainSubsystem drivetrain;

    public final LimelightSubsystem limelights[];

    public final LauncherSubsystem launcher = new LauncherSubsystem();
    public final ClimberSubsystem climber = new ClimberSubsystem();
    public final SpinDexSubsystem spindex = new SpinDexSubsystem();
    public final TurretSubsystem turret;
    public final IntakeSubsystem intake = new IntakeSubsystem();

    public final HoodSubsystem hood;

    public final Telemetry logger;

    private void resolveAllianceDependencies() {
        this.targetArc = new Arc(
                FieldConstants.BLUE_HUB_CENTER,
                Feet.of(9).in(Meters),
                Rotation2d.fromDegrees(90),
                Rotation2d.fromDegrees(180)).alliance();
        Telemetry.field.getObject("arc").setPoses(this.targetArc.getAsPoses());
    }

    public Robot(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        this.logger = new Telemetry(this);
        turret = new TurretSubsystem(drivetrain::getEstimatedPosition, drivetrain.constants);
        limelights = new LimelightSubsystem[] {
                new LimelightSubsystem(drivetrain, "limelight-fone"),
                new LimelightSubsystem(drivetrain, "limelight-ftwo"),
        };
        hood = new HoodSubsystem(turret::getMechanismPose, FieldConstants.HOOD_EXCLUSION_ZONES);

        NTable.root().set("current robot", switch (Constants.CURRENT_ROBOT) {
            case FULL_COMPBOT -> "compbot";
            case FULL_DEVBOT -> "devbot";
            default -> "???";
        });
        NTable.root().set("scheduler", CommandScheduler.getInstance());

        configureBindings();
        setupAutoChooser();

        CommandScheduler.getInstance().schedule(
                PathfindingCommand.warmupCommand().withName("Warmup Pathfinding"));

        SignalLogger.enableAutoLogging(false);
        StatusLogger.disableAutoLogging();

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
            Pose2d pose = limelights[0].getPoseEstimate();
            if (pos == null) {
                pos = limelights[1].getPoseEstimate();
            }
            if (pos != null) {
                drivetrain.resetPose(pose);
            }
            return pos;
        }, "stay in place !", true));
        commandsForAuto.put("extend climber", climber.getFullyExtendCommand());
        commandsForAuto.put("detract climber",
                climber.getFullyDetractCommand().alongWith(turret.holdRobotRel(TurretConstants.CLIMB_POS_BOT_REL)));
        commandsForAuto.put("drop intake", intake.getSlapdownCommand());
        commandsForAuto.put("run intake", intake.getIntakeForwardRollCommand());
        commandsForAuto.put("Put up Intake", intake.getLiftCommand());
        commandsForAuto.put("run spindex", spindex.getRun());
        commandsForAuto.put("dynamic launch",
                LaunchCalculator.dynamicLaunchCommand(LaunchGoal.SCORE, () -> false, hood, turret, drivetrain, launcher,
                        spindex));
        commandsForAuto.put("launch at 3000 rpm", launcher.getLaunchFuel(RPM.of(3000)));
        commandsForAuto.put("Turret track Blue Hub",
                turret.trackFieldPos(FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER)));
        commandsForAuto.put("Hood atZero", hood.runOnce(() -> hood.setHoodAngle(0)));
        commandsForAuto.put("Hood at20", hood.runOnce(() -> hood.setHoodAngle(20)));

        NamedCommands.registerCommands(commandsForAuto);

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Choose an Auto", autoChooser);
    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        setDefaultCommands();
        setupMiscTriggers();
        setupDriverBindings();
        setupSubsystemBindings();
        setupOtherBindings();
    }

    // all default commands go here
    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(
                drivetrain.joystickDriveCommand(
                        () -> driveController.getLeftX(),
                        () -> driveController.getLeftY(),
                        () -> driveController.getLeftTriggerAxis(),
                        () -> driveController.getRightTriggerAxis(),
                        () -> driveController.getHID().getYButton(),
                        () -> driveController.getHID().getStartButton()));

        turret.setDefaultCommand(turret.holdRobotRel(TurretConstants.START_POS_BOT_REL));
        hood.setDefaultCommand(hood.run(() -> hood.setHoodAngle(HoodConstants.LOWEST_ANGLE_DEGREES)));
    }

    // any trigger that isn't a button goes here
    private void setupMiscTriggers() {
        hood.exclusionZoneTrigger.whileTrue(Commands.run(() -> hood.setHoodPosition(0)));
        hood.exclusionZoneTrigger.onTrue(Commands.runOnce(() -> {
            SmartDashboard.putBoolean("in_exclusion_zone", true);
            // todo: add telemetry / debug / logging
            hood.exzSaveServoPosition();
        }).withName("enter exclusion zone"));
        hood.exclusionZoneTrigger.onFalse(Commands.runOnce(() -> {
            SmartDashboard.putBoolean("in_exclusion_zone", false);
            // todo: add telemetry / debug / logging
            double pos = hood.exzGetSavedServoPosition();
            hood.setServoPosition(pos);
        }).withName("leave exclusion zone"));

        // resets the turrets position when it engages the Hall-Effect sensor
        turret.limitTrigger.onTrue(turret.zeroCommand());
        MatchState.hubActiveTrigger
                .onTrue(Commands.runOnce(() -> driveController.setRumble(RumbleType.kBothRumble, 0.5)));
        MatchState.hubActiveTrigger
                .onFalse(Commands.runOnce(() -> driveController.setRumble(RumbleType.kBothRumble, 0)));
        driveController.setRumble(RumbleType.kBothRumble, 0);
        intake.limitEngaged.onTrue(intake.zeroSlapdownPosition());
    }

    boolean lastDroveToArc = true;

    private void setupDriverBindings() {
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        Translation2d hub = FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER);

        // @formatter:off
        // drive to and along the arc
        driveController.a().whileTrue(
            new SequentialCommandGroup(
                Commands.runOnce(() -> this.lastDroveToArc = true),
                // drive to the arc, this ends when we're at the arc
                new PIDToPose(drivetrain,
                    () -> targetArc.getShootingPose(
                        drivetrain.getEstimatedPosition().getTranslation(),
                        Rotation2d.kCW_90deg
                    ),
                    "drive to arc"),
                // if we haven't been cancelled by now, let the driver drive along the arc
                new DriveOnArc(drivetrain, () -> targetArc,
                    () -> MathUtil.applyDeadband(driveController.getLeftX(), 0.1),
                    Rotation2d.kCW_90deg)
            ).withName("drive to and on arc")
        );
        driveController.a().whileTrue(launcher.getLaunchFuel(RPM.of(3000)));
                
        // drive to the nearest ferry shoot position
        driveController.x().whileTrue(
            new SequentialCommandGroup(
                Commands.runOnce(() -> this.lastDroveToArc = false),
                hood.runOnce(() -> hood.setHoodAngle(HoodConstants.HIGHEST_ANGLE_DEGREES)),
                // drive to the nearest shooting start position
                new PIDToPose(drivetrain, () ->
                    FieldConstants.closestFerryShootPos(drivetrain.getEstimatedPosition().getTranslation()
                ),
                "drive to shooting pos")
            ).withName("drive to shooting pos")
        );
        driveController.x().whileTrue(launcher.getLaunchFuel(RPM.of(3000)));

        // set the hood angle
        driveController.b().whileTrue(
            // track the shooting target
            Commands.either(
                turret.trackFieldPos(hub),
                turret.trackFieldPosDynamic(
                    () -> FieldConstants.closestFerryTarget(drivetrain.getEstimatedPosition().getTranslation())
                ),
                () -> lastDroveToArc).alongWith(
            Commands.either(
                // get the angle from NT if we're at the arc
                hood.run(() -> hood.setHoodAngle(HoodConstants.ANGLE_AT_ARC)),
                // otherwise, shoot at the max angle
                hood.run(() -> hood.setHoodAngle(HoodConstants.HIGHEST_ANGLE_DEGREES)),
                () -> lastDroveToArc
            ))
        );
        driveController.b().whileTrue(
            new SequentialCommandGroup(
                // spin up the shooter
                launcher.getLaunchFuel(RPM.of(3000)).until(() ->
                    // are we ready?
                    launcher.atSetpoint()
                ).withTimeout(Seconds.of(2)),
                new ParallelCommandGroup(
                    // spin the spindex (and the feeder)
                    spindex.getRun(),
                    // continue spinning the shooter
                    launcher.run(launcher::usePID)
                )
            ).withName("shoot")
        );

        // spin the spindex backwards to unclog
        driveController.b().onFalse(spindex.getBackwards().withTimeout(Seconds.of(0.5)).withName("spindex backwards"));
        // stop spinning the shooter (with delay to fix unknown bug)
        driveController.b().onFalse(Commands.waitTime(Seconds.of(0.1)).andThen(launcher.getResting()).withName("stop shooter"));
        // @formatter:on

        driveController.rightBumper().whileTrue(intake.getIntakeForwardRollCommand());
        driveController.leftBumper().whileTrue(intake.getIntakeReverseRollCommand());
        driveController.povLeft().whileTrue(intake.getSlapdownCommand());
        driveController.povRight().whileTrue(intake.getLiftCommand());
    }

    double angle = 20;

    private void setupSubsystemBindings() {
        subsystemController.b().whileTrue(hood.runOnce(() -> {
            hood.exzSaveServoPosition();
            hood.setHoodPosition(0);
        }));
        subsystemController.povUp().onTrue(hood.runOnce(() -> hood.setHoodAngle(angle += 5)));
        subsystemController.povDown().onTrue(hood.runOnce(() -> hood.setHoodAngle(angle -= 5)));
        subsystemController.a().whileTrue(spindex.getBackwards());
        subsystemController.rightTrigger().whileTrue(climber.getExtendCommand());
        subsystemController.leftTrigger().whileTrue(climber.getRetractCommand().alongWith(
                turret.holdRobotRel(TurretConstants.CLIMB_POS_BOT_REL)));
    }

    double current = HoodConstants.ANGLE_AT_ARC;

    private void setupOtherBindings() {
        hood.setHoodAngle(20.0); // set hood to an initial value (make it similar to other values in this file)
        testController.b().onTrue(launcher.getLaunchFuel(RPM.of(3000))
                .until(() -> driveController.getHID().getBackButton() || launcher.atSetpoint())
                .withTimeout(Seconds.of(2)));
        testController.x().onTrue(hood.runOnce(() -> hood.setHoodAngle(HoodConstants.ANGLE_AT_ARC)));
        testController.y()
                .onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(limelights[0].getPoseEstimate())));

        testController.povUp().onTrue(Commands.runOnce(() -> hood.setHoodAngle(HoodConstants.HIGHEST_ANGLE_DEGREES)));
        testController.povDown().onTrue(Commands.runOnce(() -> hood.setHoodAngle(HoodConstants.LOWEST_ANGLE_DEGREES)));
        testController.povLeft().onTrue(Commands.runOnce(() -> hood.setHoodAngle(current -= 5)));
        testController.povRight().onTrue(Commands.runOnce(() -> hood.setHoodAngle(current += 5)));

        testController.start().whileTrue(spindex.getRun());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        NTable.updateAllSendables();

        NTable.root("telemetry").set("last drove to arc", lastDroveToArc);
    }

    @Override
    public void disabledInit() {
        driveController.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public void disabledPeriodic() {
        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUToPigeon();
        }
    }

    @Override
    public void autonomousInit() {
        resolveAllianceDependencies();

        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUMode(3);
        }

        // if (!turret.getZeroStatus()) {
        // CommandScheduler.getInstance().schedule(turret.zeroSequence());
        // }

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
        resolveAllianceDependencies();

        if (storedAuto != null) {
            storedAuto.cancel();
        }

        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUMode(3);
        }

        if (!turret.getZeroStatus()) {
            CommandScheduler.getInstance().schedule(turret.zeroSequence());
        }

        launcher.stop();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        resolveAllianceDependencies();
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
