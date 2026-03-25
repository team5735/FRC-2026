//Logic based on FRC 6328 and write-ups by the independent developer Oblarg

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Telemetry;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SpinDexSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class LaunchCalculator {
    // key: distance (m) value: angle(deg)
    private static final InterpolatingDoubleTreeMap scoreHoodMap = new InterpolatingDoubleTreeMap();
    // key: distance(m) value: evlocity(RPM)
    private static final InterpolatingDoubleTreeMap scoreSpeedMap = new InterpolatingDoubleTreeMap();
    // key: distance(m) value: Time of Flight (s)
    private static final InterpolatingDoubleTreeMap scoreTOFMap = new InterpolatingDoubleTreeMap();

    // key: distance (m) value: angle(deg)
    private static final InterpolatingDoubleTreeMap ferryHoodMap = new InterpolatingDoubleTreeMap();
    // key: distance (m) value: velocity(RPM)
    private static final InterpolatingDoubleTreeMap ferrySpeedMap = new InterpolatingDoubleTreeMap();
    // key: distance (m) value: Time of Flight (s)
    private static final InterpolatingDoubleTreeMap ferryTOFMap = new InterpolatingDoubleTreeMap();

    static {
        // TODO populate tree maps here with REAL POSITIONS

        // ALL THESE VALUES ARE TEMP FOR TESTING !!
        scoreHoodMap.put(Units.inchesToMeters(56), 8.);
        scoreHoodMap.put(Units.inchesToMeters(71.75), 15.);
        scoreHoodMap.put(Units.inchesToMeters(87.8), 15.);
        scoreHoodMap.put(Units.inchesToMeters(102.2), 20.);
        scoreHoodMap.put(Units.inchesToMeters(216), 20.);

        scoreSpeedMap.put(Units.inchesToMeters(56), 2600.);
        scoreSpeedMap.put(Units.inchesToMeters(71.75), 2650.);
        scoreSpeedMap.put(Units.inchesToMeters(87.8), 2675.);
        scoreSpeedMap.put(Units.inchesToMeters(102.2), 2800.);
        scoreSpeedMap.put(Units.inchesToMeters(117), 2900.);
        scoreSpeedMap.put(Units.inchesToMeters(132), 3050.);
        scoreSpeedMap.put(Units.inchesToMeters(147), 3200.);
        scoreSpeedMap.put(Units.inchesToMeters(163), 3350.);
        scoreSpeedMap.put(Units.inchesToMeters(177), 3450.);
        scoreSpeedMap.put(Units.inchesToMeters(192), 3650.);
        scoreSpeedMap.put(Units.inchesToMeters(207), 3850.);
        scoreSpeedMap.put(Units.inchesToMeters(216), 4000.);

        scoreTOFMap.put(0., 0.);
        scoreTOFMap.put(50., 5.);
    }

    private static final LinearFilter turretVelFiler = LinearFilter.movingAverage(50);
    private static final Timer timer = new Timer();

    private static final double MIN_SCORE_DIST_M = Units.inchesToMeters(56);
    private static final double MAX_SCORE_DIST_M = Units.inchesToMeters(216); // TODO update

    private static boolean allianceKnown = false;
    private static boolean isBlue = true;
    private static Angle oldTurretAngle;
    private static LaunchParams cachedParams = new LaunchParams(
            false,
            Rotations.of(0),
            RotationsPerSecond.of(0),
            Degrees.of(HoodConstants.LOWEST_ANGLE_DEGREES),
            RPM.of(0));

    public static final double TOF_TOLERANCE = 0.05;
    public static final double DRIVETRAIN_VELOCITY_SCALING = 0.5;
    public static final double RECALC_PERIOD = 0.02;

    public record LaunchParams(
            boolean isValid,
            Angle turretAngle, // robot-relative
            AngularVelocity turretVelocity,
            Angle hoodAngle,
            AngularVelocity flywheelVelocity) {
    }

    public enum LaunchGoal {
        SCORE,
        PASS
    }

    public static void calculate(LaunchGoal goal, DrivetrainSubsystem drivetrain, ChassisSpeeds robotVel,
            Pose2d turretPose) {
        if (!allianceKnown) {
            if (DriverStation.getAlliance().isPresent()) {
                allianceKnown = true;
                isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);
            }
        }

        Translation2d launchTarget = switch (goal) {
            case SCORE -> isBlue ? FieldConstants.BLUE_HUB_CENTER
                    : FieldConstants.redElement(FieldConstants.BLUE_HUB_CENTER);
            case PASS -> new Translation2d(); // TODO set pass targets
        };

        double robotAngle = drivetrain.getEstimatedPosition().getRotation().getRadians();

        Translation2d launchOrigin = turretPose.getTranslation();

        double turretVelX = robotVel.vxMetersPerSecond + robotVel.omegaRadiansPerSecond
                * (drivetrain.constants.getRobotToTurretCenter().getY() * Math.cos(robotAngle)
                        - drivetrain.constants.getRobotToTurretCenter().getX() * Math.sin(robotAngle));

        SmartDashboard.putNumber("launchCalc/turret_vx", turretVelX);

        double turretVelY = robotVel.vyMetersPerSecond + robotVel.omegaRadiansPerSecond
                * (drivetrain.constants.getRobotToTurretCenter().getX() * Math.cos(robotAngle)
                        - drivetrain.constants.getRobotToTurretCenter().getY() * Math.sin(robotAngle));

        SmartDashboard.putNumber("launchCalc/turret_vy", turretVelX);

        double launchDist = launchTarget.getDistance(launchOrigin); // naive initial estimate

        SmartDashboard.putNumber("launchCalc/naive_dist", launchDist);
        double timeOfFlight;
        Translation2d lookaheadOrigin = launchOrigin;
        double oldTOF = 100000; // arbitrarily large; be glad I didn't make this 67676767
        for (int i = 0; i < 20; i++) {
            timeOfFlight = scoreTOFMap.get(launchDist);
            double deltaX = turretVelX * timeOfFlight;
            double deltaY = turretVelY * timeOfFlight;

            lookaheadOrigin = launchOrigin.plus(new Translation2d(deltaX, deltaY));
            launchDist = launchTarget.getDistance(lookaheadOrigin);

            if (MathUtil.isNear(oldTOF, timeOfFlight, TOF_TOLERANCE)) {
                break;
            }

            oldTOF = timeOfFlight;
        }

        launchOrigin = lookaheadOrigin;

        SmartDashboard.putNumber("launchCalc/wise_dist", launchDist);

        Rotation2d turretRot = launchTarget.minus(launchOrigin).getAngle()
                .minus(drivetrain.getEstimatedPosition().getRotation());

        Angle turretAngle = turretRot.getMeasure();

        Telemetry.field.getObject("launch origin").setPose(turretPose);
        Telemetry.field.getObject("launch lookahead")
                .setPose(new Pose2d(launchOrigin,
                        turretRot.plus(drivetrain.getEstimatedPosition().getRotation())));

        if (oldTurretAngle == null)
            oldTurretAngle = turretAngle;

        AngularVelocity turretVel = RotationsPerSecond
                .of(turretVelFiler.calculate(
                        turretAngle.minus(oldTurretAngle).in(Rotations) / 0.02));

        timer.reset();

        oldTurretAngle = turretAngle;
        Angle hoodAngle = Degrees.of(scoreHoodMap.get(launchDist));

        AngularVelocity flywheelVel = RPM.of(scoreSpeedMap.get(launchDist));
        SmartDashboard.putNumber("launchCalc/turret_angle_rot", turretAngle.in(Rotations));
        SmartDashboard.putNumber("launchCalc/turret_vel_rps", turretVel.in(RotationsPerSecond));
        SmartDashboard.putNumber("launchCalc/hood_angle_deg", hoodAngle.in(Degrees));
        SmartDashboard.putNumber("launchCalc/flywheel_vel_rpm", flywheelVel.in(RPM));

        cachedParams = new LaunchParams(
                (launchDist >= MIN_SCORE_DIST_M) && (launchDist <= MAX_SCORE_DIST_M), // TODO - set exclusion zones
                turretAngle,
                turretVel,
                hoodAngle,
                flywheelVel);
    }

    public static LaunchParams getCachedParams() {
        return cachedParams;
    }

    private static Command reCalcParams(LaunchGoal goal, TurretSubsystem turret, DrivetrainSubsystem drivetrain) {
        return Commands.run(() -> calculate(goal, drivetrain, drivetrain.getChassisSpeedsFieldRel(),
                turret.getMechanismPose()));
    }

    public static Command dynamicLaunchCommand(LaunchGoal goal, BooleanSupplier override,
            HoodSubsystem hood, TurretSubsystem turret, DrivetrainSubsystem drivetrain,
            LauncherSubsystem launcher, SpinDexSubsystem spindex) {
        return Commands.runOnce(() -> timer.reset()).andThen(Commands.parallel(
                reCalcParams(goal, turret, drivetrain),
                hood.getDynamicTracking(() -> getCachedParams().hoodAngle),
                turret.trackRobotRelWithVelocity(() -> getCachedParams().turretAngle,
                        () -> getCachedParams().turretVelocity),
                launcher.getDynamicLaunch(() -> getCachedParams().flywheelVelocity),
                spindex.idle().until(() -> {
                    LaunchParams params = getCachedParams();
                    return (params.isValid
                            && MathUtil.isNear(params.hoodAngle.in(Degrees),
                                    hood.getHoodAngle(),
                                    HoodConstants.DYNAMIC_TOLERANCE_DEGREES)
                            && turret.getAngle().isNear(params.turretAngle,
                                    TurretConstants.TOLERANCE.plus(Degrees.of(0.5)))
                            && MathUtil.isNear(params.flywheelVelocity.in(RPM), launcher.getRPM(), LauncherConstants.RPM_TOLERANCE)
                            || override.getAsBoolean());
                }).andThen(spindex.getInformedRun(() -> !TurretConstants.isInDeadZone(getCachedParams().turretAngle) || override.getAsBoolean()))));
    }

    public static Command dynamicLaunchTeleop(CommandXboxController controller, BooleanSupplier override,
            LaunchGoal goal,
            HoodSubsystem hood, TurretSubsystem turret, DrivetrainSubsystem drivetrain,
            LauncherSubsystem launcher, SpinDexSubsystem spindex) {
        return dynamicLaunchCommand(goal, override, hood, turret, drivetrain, launcher, spindex)
                .alongWith(drivetrain.joystickDriveCommand(
                        () -> controller.getLeftX() * DRIVETRAIN_VELOCITY_SCALING,
                        () -> controller.getLeftY() * DRIVETRAIN_VELOCITY_SCALING,
                        () -> controller.getLeftTriggerAxis() * DRIVETRAIN_VELOCITY_SCALING,
                        () -> controller.getRightTriggerAxis() * DRIVETRAIN_VELOCITY_SCALING,
                        () -> controller.getHID().getBButton(),
                        () -> controller.getHID().getStartButton()));
    }
}
