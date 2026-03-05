//Logic based on FRC 6328 and write-ups by the independent developer Oblarg

package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;

public class LaunchCalculator {
    private static final InterpolatingTreeMap<Double, Rotation2d> scoreHoodMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap scoreSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap scoreTOFMap = new InterpolatingDoubleTreeMap();

    private static final InterpolatingTreeMap<Double, Rotation2d> passHoodMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap passSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap passTOFMap = new InterpolatingDoubleTreeMap();

    static {
        // TODO populate tree maps here
    }

    private static final LinearFilter turretVelFiler = LinearFilter.movingAverage(5);
   
    private static boolean allianceKnown = false;
    private static boolean isBlue = true;
    private static Angle oldTurretAngle;

    public static final double TOF_TOLERANCE = 0.05;

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

    public LaunchParams calculate(LaunchGoal goal, Pose2d robotPose, ChassisSpeeds robotVel, Pose2d turretPose) {
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

        Translation2d launchOrigin = turretPose.getTranslation();
        double turretVelX = robotVel.vxMetersPerSecond
                + robotVel.omegaRadiansPerSecond
                        * (RobotConstants.ROBOT_TO_TURRET_CENTER.getY() * Math.cos(robotPose.getRotation().getRadians())
                                - RobotConstants.ROBOT_TO_TURRET_CENTER.getX()
                                        * Math.sin(robotPose.getRotation().getRadians()));
        double turretVelY= robotVel.vyMetersPerSecond
                + robotVel.omegaRadiansPerSecond
                        * (RobotConstants.ROBOT_TO_TURRET_CENTER.getX() * Math.cos(robotPose.getRotation().getRadians())
                                - RobotConstants.ROBOT_TO_TURRET_CENTER.getY()
                                        * Math.sin(robotPose.getRotation().getRadians()));

        double launchDist = launchTarget.getDistance(launchOrigin); // naive initial estimate

        double timeOfFlight;
        double oldTOF = 100000; // arbitrarily large; be glad I didn't make this 67676767
        for (int i = 0; i < 20; i++) {
            timeOfFlight = scoreTOFMap.get(launchDist);
            double deltaX = turretVelX * timeOfFlight;
            double deltaY = turretVelY * timeOfFlight;

            launchOrigin = launchOrigin.plus(new Translation2d(deltaX, deltaY));
            launchDist = launchTarget.getDistance(launchOrigin);

            if(MathUtil.isNear(oldTOF, timeOfFlight, TOF_TOLERANCE)){
                break;
            }

            oldTOF = timeOfFlight;
        }

        Angle turretAngle = launchTarget.minus(turretPose.getTranslation()).getAngle().minus(robotPose.getRotation()).getMeasure();

        if(oldTurretAngle == null) 
            oldTurretAngle = turretAngle;

        AngularVelocity turretVel = RotationsPerSecond.of(turretVelFiler.calculate(turretAngle.minus(oldTurretAngle).in(Rotations) / 0.02));
        
        return new LaunchParams(
                false,
                turretAngle,
                turretVel,
                scoreHoodMap.get(launchDist).getMeasure(),
                RPM.of(scoreSpeedMap.get(launchDist)));
    }
}