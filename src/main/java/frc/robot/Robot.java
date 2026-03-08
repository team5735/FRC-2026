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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveOnArc;
import frc.robot.commands.drivetrain.PIDToPose;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FuelLauncherSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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

    public final Arc targetArc = new Arc(FieldConstants.BLUE_HUB_CENTER,
            Feet.of(7.5).in(Meters),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(270));

    public final DrivetrainSubsystem drivetrain;

    public final LimelightSubsystem limelights[];

    public final FuelLauncherSubsystem launcher = new FuelLauncherSubsystem();
    public final ClimberSubsystem climber = new ClimberSubsystem();
    public final SpinDexSubsystem spindex = new SpinDexSubsystem();
    public final TurretSubsystem turret;
    public final IntakeSubsystem intake = new IntakeSubsystem();

    public final HoodSubsystem hood;

    public final Telemetry logger;

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

        setupDriverBindings();
        setupSubsystemBindings();
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

        driveController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driveController.x()
                .whileTrue(new PIDToPose(drivetrain,
                        () -> targetArc.getPoseFacingCenter(
                                targetArc.nearestPointOnArc(drivetrain.getEstimatedPosition().getTranslation())),
                        "drive to arc")

                        .andThen(new DriveOnArc(drivetrain, targetArc,
                                () -> MathUtil.applyDeadband(driveController.getLeftX(), 0.1))));

        // @formatter:off
        driveController.a().whileTrue(
            // track the hub the entire time
            turret.trackFieldPos(
                FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER)
            ).alongWith(

                // prepare to shoot fuel
                new ParallelCommandGroup(
                    // drive to the arc
                    new PIDToPose(
                        drivetrain,
                        () -> targetArc.getPoseFacingCenter(targetArc.nearestPointOnArc(
                            drivetrain.getEstimatedPosition().getTranslation()
                        )),
                        "drive to arc (shoot)"
                    ),

                    // spin up the shooter
                    launcher.getLaunchFuel(RPM.of(3000))
                        .until(() ->
                            // if the back button is being pressed, skip waiting for the setpoint
                            driveController.getHID().getBackButton() || launcher.atSetpoint())
                        // another failsafe: if we're taking too long to spin up, assume something is wrong
                        .withTimeout(Seconds.of(2)),

                    // set the hood to the right position. this does not have an until
                    // because we don't have a way to get the current servo's position
                    hood.runOnce(() -> hood.setHoodAngle(HoodConstants.ANGLE_AT_ARC))

                // shoot fuel. this only executes once:
                // - robot is in the right place
                // - the shooter is up to speed
                ).andThen(new ParallelCommandGroup(
                    // spin the spindex and the feeder
                    spindex.getRun(),
                    // make sure the shooter keeps spinning
                    launcher.run(launcher::usePID),
                    // let the driver drive along the arc that we drove to with the left
                    // stick's X axis
                    new DriveOnArc(drivetrain, targetArc,
                        () -> MathUtil.applyDeadband(driveController.getLeftX(), 0.1))
                ))
            )
        );
        // @formatter:on

        // when the button is released, spin the spindex backwards for a bit to unclog
        // any possible clogs
        driveController.a().onFalse(spindex.getBackwards().withTimeout(Seconds.of(0.5)));

        driveController.rightBumper().whileTrue(intake.getIntakeForwardRollCommand());
        driveController.leftBumper().whileTrue(intake.getIntakeReverseRollCommand());
        // I wasn't sure what values to give as targetPosition for these, I guessed 12.5
        driveController.povLeft().onTrue(intake.getSlapdownCommand());
        driveController.povRight().onTrue(intake.getLiftCommand());
    }

    private void setupSubsystemBindings() {
        subsystemController.a().whileTrue(climber.getClimbUpCommand());
        subsystemController.b().whileTrue(climber.getClimbDownCommand());
        subsystemController.rightTrigger().whileTrue(climber.getClimbUpCommand());
        subsystemController.leftTrigger().whileTrue(climber.getClimbDownCommand());
    }

    private void setupOtherBindings() {
        turret.setDefaultCommand(turret.holdRobotRel(TurretConstants.START_POS_BOT_REL));
        turret.limitTrigger.onTrue(turret.zeroCommand()); // resets the turrets position when it engages the Hall-Effect
                                                          // sensor
        launcher.setDefaultCommand(launcher.getLaunchFuel(RPM.of(0)));

        // test individual parts of the a button command monster
        testController.a().whileTrue(new PIDToPose(
                drivetrain,
                () -> targetArc.getPoseFacingCenter(targetArc.nearestPointOnArc(
                        drivetrain.getEstimatedPosition().getTranslation())),
                "drive to arc (shoot)")
                .andThen(new DriveOnArc(drivetrain, targetArc,
                        () -> MathUtil.applyDeadband(driveController.getLeftX(), 0.1))));

        testController.b().onTrue(launcher.getLaunchFuel(RPM.of(3000))
                .until(() -> driveController.getHID().getBackButton() || launcher.atSetpoint())
                .withTimeout(Seconds.of(2)));
        testController.x().onTrue(hood.runOnce(() -> hood.setHoodAngle(HoodConstants.ANGLE_AT_ARC)));
        testController.y()
                .onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(limelights[0].new PoseEstimate().pose2d)));

        testController.povUp().onTrue(Commands.runOnce(() -> hood.setHoodPosition(0.7)));
        testController.povDown().onTrue(Commands.runOnce(() -> hood.setHoodPosition(0.3)));
        testController.povLeft().onTrue(turret.runOnce(() -> turret.remakePID()));

        testController.rightBumper().whileTrue(turret.testForward());
        testController.leftBumper().whileTrue(turret.testReverse());

        subsystemController.a().whileTrue(spindex.getRun());
    }

    @Override
    public void robotPeriodic() {
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
