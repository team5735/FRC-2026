package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import java.util.ArrayList;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.PIDToPose;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.robot.CompbotTunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SpinDexSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.NTable;

public class InterpolationRobot extends TimedRobot {
    static class DistanceDependentParams {
        double hoodAngle;
        double launcherRPM;
        double spindexVolts;
        double feederVolts;

        double distance;
        NTable table;

        public DistanceDependentParams(double hoodAngle, double launcherRPM, double spindexVolts, double feederVolts,
                double distance) {
            this.hoodAngle = hoodAngle;
            this.launcherRPM = launcherRPM;
            this.spindexVolts = spindexVolts;
            this.feederVolts = feederVolts;
            this.distance = distance;
            this.table = NTable.root("interpolation").sub(String.valueOf(distance));
            publishToNT();
        }

        public DistanceDependentParams(double distance) {
            this(HoodConstants.ANGLE_AT_ARC, LauncherConstants.DEFAULT_SETPOINT.in(RPM), -3, -6, distance);
        }

        void publishToNT() {
            table.set("hood angle", hoodAngle);
            table.set("launcher rpm", launcherRPM);
            table.set("spindexer voltage", spindexVolts);
            table.set("feeder voltage", feederVolts);
        }

        void updateFromNT() {
            hoodAngle = table.getDouble("hood angle");
            launcherRPM = table.getDouble("launcher rpm");
            spindexVolts = table.getDouble("spindexer voltage");
            feederVolts = table.getDouble("feeder voltage");
        }

        // fraction goes from 0 to 1
        static DistanceDependentParams interpolate(double fraction, DistanceDependentParams prev,
                DistanceDependentParams next) {
            return new DistanceDependentParams(
                    prev.hoodAngle + (next.hoodAngle - prev.hoodAngle) * fraction,
                    prev.launcherRPM + (next.launcherRPM - prev.launcherRPM) * fraction,
                    prev.spindexVolts + (next.spindexVolts - prev.spindexVolts) * fraction,
                    prev.feederVolts + (next.feederVolts - prev.feederVolts) * fraction,
                    prev.distance + (next.distance - prev.distance) * fraction);
        }
    }

    ArrayList<DistanceDependentParams> configs = new ArrayList<>();

    // add configs to here when we're confident with them
    void insertSavedConfigs() {
        configs.add(new DistanceDependentParams(getDistance()));
    }

    DrivetrainSubsystem drivetrain = CompbotTunerConstants.createDrivetrain();

    LimelightSubsystem limelights[] = new LimelightSubsystem[] {
            new LimelightSubsystem(drivetrain, "limelight-fone"),
            new LimelightSubsystem(drivetrain, "limelight-ftwo"),
    };

    LauncherSubsystem launcher = new LauncherSubsystem();
    SpinDexSubsystem spindex = new SpinDexSubsystem();
    TurretSubsystem turret = new TurretSubsystem(drivetrain::getEstimatedPosition, drivetrain.constants);
    IntakeSubsystem intake = new IntakeSubsystem();
    HoodSubsystem hood = new HoodSubsystem(turret::getMechanismPose, FieldConstants.HOOD_EXCLUSION_ZONES);

    Telemetry logger = new Telemetry(drivetrain, turret);

    Translation2d target = FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER);

    public InterpolationRobot() {
        drivetrain.registerTelemetry(logger::telemeterize);

        insertSavedConfigs();

        NTable.root().set("current robot", "interpolation");
        NTable.root().set("scheduler", CommandScheduler.getInstance());

        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUToPigeon();
        }

        DriverStation.silenceJoystickConnectionWarning(true);
        SignalLogger.enableAutoLogging(false);
        StatusLogger.disableAutoLogging();

        configureBindings();
    }

    CommandXboxController driveController = new CommandXboxController(Constants.DRIVE_CONTROLLER_PORT);

    double getDistance() {
        return Math.round(turret.getMechanismPose().getTranslation().getDistance(target) * 10.0) / 10.0;
    }

    DistanceDependentParams getOrInterpolateParams(double distance) {
        this.configs.forEach(params -> params.updateFromNT());
        this.configs.sort((a, b) -> Double.compare(a.distance, b.distance));
        // search in configs to find where dist is or would be
        int i = 0;
        while (i < this.configs.size() && this.configs.get(i).distance < distance) {
            i++;
        }
        System.out.println(i);
        DistanceDependentParams prev = null;
        if (i > 0) {
            prev = this.configs.get(i - 1);
        }
        DistanceDependentParams next = this.configs.get(i);
        if (prev == null) {
            return next;
        }
        double interp = (distance - prev.distance) / (next.distance - prev.distance);
        return DistanceDependentParams.interpolate(interp, prev, next);
    }

    DistanceDependentParams currentConfig;

    void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.joystickDriveCommand(
                        () -> driveController.getLeftX(),
                        () -> driveController.getLeftY(),
                        () -> driveController.getLeftTriggerAxis(),
                        () -> driveController.getRightTriggerAxis(),
                        () -> driveController.getHID().getYButton(),
                        () -> driveController.getHID().getStartButton()));

        driveController.x().onTrue(Commands.runOnce(() -> {
            if (this.configs.stream().anyMatch(params -> params.distance == getDistance())) {
                System.out.println("config already exists");
            } else if (this.configs.size() == 1) {
                System.out.println("added new config (defaults)");
                this.configs.add(new DistanceDependentParams(getDistance()));
            } else {
                System.out.println("added new config via interpolating");
                this.configs.add(getOrInterpolateParams(getDistance()));
            }
        }));

        // @formatter:off
        driveController.a().whileTrue(
            new PIDToPose(drivetrain, () -> new Pose2d(
                drivetrain.getEstimatedPosition().getTranslation(),
                this.target.minus(drivetrain.getEstimatedPosition().getTranslation()).getAngle().plus(Rotation2d.kCW_90deg)
            ), "face 90° off of hub")
        );
        driveController.b().onTrue(new SequentialCommandGroup(
            Commands.runOnce(() -> currentConfig = getOrInterpolateParams(getDistance())),
            launcher.getLaunchFuelSupplier(() -> currentConfig.launcherRPM).withTimeout(2),
            hood.runOnce(() -> hood.setHoodAngle(currentConfig.hoodAngle)),
            new ParallelCommandGroup(
                spindex.getRunSupplier(() -> currentConfig.spindexVolts, () -> currentConfig.feederVolts),
                launcher.run(launcher::usePID)
            )
        ));
        // @formatter:on
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        NTable.updateAllSendables();
        NTable.root("interpolation").set("turret distance to hub",
                turret.getMechanismPose().getTranslation().getDistance(target));
        NTable.root("interpolation").set("turret distance to hub (rounded)", getDistance());
    }

    @Override
    public void teleopInit() {
        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUMode(3);
        }
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
