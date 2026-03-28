package frc.robot.constants.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public class CompbotConstants implements RobotConstants {
    @Override
    public PIDConstants getAutoPosConstants() {
        return new PIDConstants(12.5, 0); // TODO
    }

    @Override
    public PIDConstants getAutoRotConstants() {
        return new PIDConstants(10, 0); // TODO
    }

    @Override
    public double getRotKs() {
        return 0.17285;
    }

    @Override
    public double getRotKv() {
        return 4.4734;
    }

    @Override
    public double getRotKa() {
        return 0.39386;
    }

    @Override
    public double getOpenDriveKv() {
        return 1.6983;
    }

    @Override
    public double getOpenDriveKa() {
        return 0.24867;
    }

    @Override
    public Distance getRobotTotalWidth() {
        return Inches.of(25).plus(getBumperWidth().times(2));
    }

    @Override
    public Distance getRobotTotalLength() {
        return Inches.of(29.5).plus(getBumperWidth().times(2));
    }

    @Override
    public Distance getPigeonToRobotFront() {
        return getRobotTotalLength().div(2);
    }

    @Override
    public PathConstraints getPathFollowConstraints() {
        return new PathConstraints(
                MetersPerSecond.of(2),
                MetersPerSecondPerSecond.of(0.5),
                RotationsPerSecond.of(0.25),
                RotationsPerSecondPerSecond.of(0.25));
    }

    @Override
    public LinearVelocity getDefaultSpeed() {
        return MetersPerSecond.of(3);
    }

    @Override
    public LinearVelocity getSlowSpeed() {
        return MetersPerSecond.of(0.6);
    }

    @Override
    public LinearVelocity getTurboSpeed() {
        return MetersPerSecond.of(5);
    }

    @Override
    public AngularVelocity getDefaultRotationalRate() {
        return DegreesPerSecond.of(180);
    }

    @Override
    public AngularVelocity getSlowRotationalRate() {
        return DegreesPerSecond.of(27);
    }

    @Override
    public AngularVelocity getTurboRotationalRate() {
        return DegreesPerSecond.of(270);
    }

    @Override
    public Mass getRobotMass() {
        return Kilograms.of(56.85);
    }

    @Override
    public Distance getDrivetrainWidth() {
        return Inches.of(29.5);
    }

    @Override
    public double getRobotMoiKgxMxM() {
        return getRobotMass().in(Kilograms) * getDrivetrainWidth().in(Meters) / 2 * getRotKa()
                / CompbotTunerConstants.DEFAULT_DRIVE_CONSTANTS.kA;
    }

    @Override
    public double getCoefficientOfFriction() {
        return 1.5;
    }

    private final RobotConfig config = new RobotConfig(
            getRobotMass().in(Kilograms),
            getRobotMoiKgxMxM(),
            new ModuleConfig(
                    CompbotTunerConstants.WHEEL_RADIUS.in(Meters),
                    CompbotTunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond),
                    getCoefficientOfFriction(),
                    DCMotor.getKrakenX60(1).withReduction(CompbotTunerConstants.DRIVE_GEAR_RATIO),
                    60,
                    1),
            new Translation2d(CompbotTunerConstants.FRONT_LEFT_XPOS, CompbotTunerConstants.FRONT_LEFT_YPOS),
            new Translation2d(CompbotTunerConstants.FRONT_RIGHT_XPOS, CompbotTunerConstants.FRONT_RIGHT_YPOS),
            new Translation2d(CompbotTunerConstants.BACK_LEFT_XPOS, CompbotTunerConstants.BACK_LEFT_YPOS),
            new Translation2d(CompbotTunerConstants.BACK_RIGHT_XPOS, CompbotTunerConstants.BACK_RIGHT_YPOS));

    @Override
    public RobotConfig getConfig() {
        return config;
    }

    public Translation2d getPigeonToCenterOfRotation() {
        return new Translation2d(Inches.of(0), Inches.of(0));
    }

    public Translation2d getRobotToTurretCenter() {
        return new Translation2d(Inches.of(-4.245), Inches.of(6.5));
    }

    public Distance getBumperWidth() {
        return Inches.of(3.5); // TODO
    }
}
