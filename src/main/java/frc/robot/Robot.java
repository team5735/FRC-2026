// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.util.Timer;
import frc.robot.util.TunablePIDController;

public class Robot extends TimedRobot {
    public final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    public final CommandXboxController subsystemController = new CommandXboxController(
            Constants.SUBSYSTEM_CONTROLLER_PORT);

    public final CommandXboxController testController = new CommandXboxController(
            Constants.TEST_CONTROLLER_PORT);

    public final DrivetrainSubsystem drivetrain;

    public final LimelightSubsystem limelights[];

    public final LauncherSubsystem launcher = new LauncherSubsystem();
    public final ClimberSubsystem climber = new ClimberSubsystem();
    public final SpinDexSubsystem spindex = new SpinDexSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final TurretSubsystem turret;
    public final HoodSubsystem hood;

    public final Telemetry logger;

    public Robot(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        turret = new TurretSubsystem(drivetrain::getEstimatedPosition, drivetrain.constants);
        this.logger = new Telemetry(drivetrain, turret);
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
<<<<<<< HEAD
        commandsForAuto.put("start intake", intake.runOnce(() -> intake.forwardRoll()));
        commandsForAuto.put("stop intake", intake.runOnce(() -> intake.stopRoll()));
=======
        commandsForAuto.put("Stop Intake", intake.getStopRollCommand());
>>>>>>> 064035b (delelted all autos - fixed them)
        commandsForAuto.put("Put up Intake", intake.getLiftCommand());
        commandsForAuto.put("run spindex", spindex.getRun());
        commandsForAuto.put("dynamic launch",
                LaunchCalculator.dynamicLaunchAuto(LaunchGoal.SCORE, hood, turret, drivetrain, launcher,
                        spindex));
        commandsForAuto.put("launch at 3000 rpm", launcher.getLaunchFuel(RPM.of(3000)));
        commandsForAuto.put("wait for shooter", Commands.waitUntil(() -> launcher.atSetpoint()));
        commandsForAuto.put("Turret track Blue Hub",
                turret.trackFieldPos(FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER)));
        commandsForAuto.put("Hood atZero", hood.runOnce(() -> hood.setHoodAngle(0)));
        commandsForAuto.put("hood 21", hood.runOnce(() -> hood.setHoodAngle(21)));


        NamedCommands.registerCommands(commandsForAuto);

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Choose an Auto", autoChooser);
    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);
        fillMaps();

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

        // turret.setDefaultCommand(turret.holdRobotRel(TurretConstants.START_POS_BOT_REL));
        hood.setHoodPosition(0); // on init, lower hood and set servo position in code
        hood.setDefaultCommand(hood.run(() -> hood.setHoodAngle(HoodConstants.LOWEST_ANGLE_DEGREES)));
        launcher.setDefaultCommand(launcher.getResting());
    }

    // any trigger that isn't a button goes here
    private void setupMiscTriggers() {
        turret.zeroTrigger.onTrue(turret.zeroCommand());
        // hood.exclusionZoneTri/aagger.whileTrue(Commands.run(() -> hood.setHoodPosition(0)));
        // hood.exclusionZoneTrigger.onTrue(Commands.runOnce(() -> {
        //     SmartDashboard.putBoolean("in_exclusion_zone", true);
        //     // todo: add telemetry / debuglogging
        //     hood.exzSaveServoPosition();
        // }).withName("enter exclusion zone"));
        // hood.exclusionZoneTrigger.onFalse(Commands.runOnce(() -> {
        //     SmartDashboard.putBoolean("in_exclusion_zone", false);
        //     // todo: add telemetry / debug / logging
        //     double pos = hood.exzGetSavedServoPosition();
        //     hood.setServoPosition(pos);
        // }).withName("leave exclusion zone"));

        // turret.limitTrigger.onTrue(turret.zeroCommand());
        MatchState.hubActiveTrigger
                .onFalse(Commands.runOnce(() -> driveController.setRumble(RumbleType.kBothRumble, 0)));
        driveController.setRumble(RumbleType.kBothRumble, 0);
        intake.limitEngaged.onTrue(intake.zeroSlapdownPosition());
    }

    TunablePIDController pidThetaFaceHub = new TunablePIDController("joystick theta");

    InterpolatingDoubleTreeMap distanceToRpm = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap distanceToAngle = new InterpolatingDoubleTreeMap();

    private void fillMaps() {
        // @formatter:off
        distanceToRpm  .put(Inches.of( 56.00).in(Meters), 2600.0);
        distanceToRpm  .put(Inches.of( 71.75).in(Meters), 2650.0);
        distanceToRpm  .put(Inches.of( 87.80).in(Meters), 2675.0);
        distanceToRpm  .put(Inches.of(102.20).in(Meters), 2800.0);
        distanceToRpm  .put(Inches.of(117.00).in(Meters), 2900.0);
        distanceToRpm  .put(Inches.of(132.00).in(Meters), 3050.0);
        distanceToRpm  .put(Inches.of(147.00).in(Meters), 3200.0);
        distanceToRpm  .put(Inches.of(163.00).in(Meters), 3350.0);
        distanceToRpm  .put(Inches.of(177.00).in(Meters), 3450.0);
        distanceToRpm  .put(Inches.of(192.00).in(Meters), 3650.0);
        distanceToRpm  .put(Inches.of(207.00).in(Meters), 3850.0);
        distanceToRpm  .put(Inches.of(216.00).in(Meters), 4000.0);

        //                                                degrees
        distanceToAngle.put(Inches.of( 56.00).in(Meters),    8.0);
        distanceToAngle.put(Inches.of( 71.75).in(Meters),   15.0);
        distanceToAngle.put(Inches.of( 87.80).in(Meters),   15.0);
        distanceToAngle.put(Inches.of(102.20).in(Meters),   20.0);
        distanceToAngle.put(Inches.of(117.00).in(Meters),   20.0);
        distanceToAngle.put(Inches.of(132.00).in(Meters),   20.0);
        distanceToAngle.put(Inches.of(147.00).in(Meters),   20.0);
        distanceToAngle.put(Inches.of(163.00).in(Meters),   20.0);
        distanceToAngle.put(Inches.of(177.00).in(Meters),   20.0);
        distanceToAngle.put(Inches.of(192.00).in(Meters),   20.0);
        distanceToAngle.put(Inches.of(207.00).in(Meters),   20.0);
        distanceToAngle.put(Inches.of(216.00).in(Meters),   20.0);
        // @formatter:on
    }

    private void setupDriverBindings() {
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        Command unclogSpindex = spindex.getBackwards().withTimeout(0.5);

        driveController.a().whileTrue(new PIDToPose(drivetrain, () -> {
            Translation2d drivetrainPos = drivetrain.getEstimatedPosition().getTranslation();
            Rotation2d drivetrainToHub = FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER).minus(drivetrainPos).getAngle();
            return new Pose2d(drivetrainPos, drivetrainToHub.plus(Rotation2d.kCCW_90deg));
        }, "face hub (backup)"));

        driveController.b().whileTrue(LaunchCalculator.dynamicLaunchTeleop(driveController, LaunchGoal.SCORE, () -> false, hood, turret, drivetrain, launcher, spindex));
        driveController.b().onFalse(unclogSpindex);

        driveController.x().whileTrue(LaunchCalculator.dynamicLaunchTeleop(driveController, LaunchGoal.FERRY, () -> false, hood, turret, drivetrain, launcher, spindex));
        driveController.x().onFalse(unclogSpindex);

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

    private void setupOtherBindings() {
        testController.b().onTrue(launcher.getLaunchFuel(RPM.of(3000))
                .until(() -> driveController.getHID().getBackButton() || launcher.atSetpoint())
                .withTimeout(Seconds.of(2)));
        testController.x().onTrue(hood.runOnce(() -> hood.setHoodAngle(HoodConstants.ANGLE_AT_ARC)));
        testController.y()
                .onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(limelights[0].getPoseEstimate())));

        testController.povUp().onTrue(Commands.runOnce(() -> hood.setHoodAngle(HoodConstants.HIGHEST_ANGLE_DEGREES)));
        testController.povDown().onTrue(Commands.runOnce(() -> hood.setHoodAngle(HoodConstants.LOWEST_ANGLE_DEGREES)));
        testController.start().whileTrue(spindex.getRun());
    }

    @Override
    public void robotPeriodic() {
        var _T = new Timer("");
        CommandScheduler.getInstance().run();
        var _TT = new Timer("Robot.robotPeriodic.updateAllSendables");
        NTable.updateAllSendables();
        _TT.toc();
        _T.toc();
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
        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUMode(3);
        }

        if (!turret.getZeroStatus()) {
        CommandScheduler.getInstance().schedule(turret.zeroSequence());
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
