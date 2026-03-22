package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.robot.CompbotTunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.NTable;
import frc.robot.util.Timer;

public class Telemetry {
    // Mechanisms to represent the swerve module states
    private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
    };
    // A direction and length changing ligament for speed representation
    private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
            moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    // A direction changing and length constant ligament for module direction
    private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
            moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] poseArray = new double[3];
    private final double[] moduleStatesArray = new double[8];
    private final double[] moduleTargetsArray = new double[8];

    public static final Field2d field = new Field2d();

    private DrivetrainSubsystem drivetrain;
    private TurretSubsystem turret;

    private final NTable table = NTable.root().sub("telemetry");
    private final NTable stateTable = table.sub("drive state");
    private final NTable moduleTable = table.sub("modules");

    Telemetry(DrivetrainSubsystem drivetrain, TurretSubsystem turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        field.getRobotObject().setPose(new Pose2d());
        field.getObject("ferry target 1").setPose(new Pose2d(FieldConstants.FERRY_TARGET_1, Rotation2d.kZero));
        field.getObject("ferry target 2").setPose(new Pose2d(FieldConstants.FERRY_TARGET_2, Rotation2d.kZero));
        field.getObject("ferry source 1").setPose(FieldConstants.FERRY_SHOOT_POS_1);
        field.getObject("ferry source 2").setPose(FieldConstants.FERRY_SHOOT_POS_2);
        table.set("field", field);

        double cx = FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_LEFT.getCenter().getX();
        double cy = FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_LEFT.getCenter().getY();
        double xw2 = FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_LEFT.getXWidth() / 2.0;
        double yw2 = FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_LEFT.getYWidth() / 2.0;

        field.getObject("exz").setPoses(new Pose2d[] {
                new Pose2d(cx - xw2, cy - yw2, Rotation2d.kZero),
                new Pose2d(cx - xw2, cy + yw2, Rotation2d.kZero),
                new Pose2d(cx + xw2, cy + yw2, Rotation2d.kZero),
                new Pose2d(cx + xw2, cy - yw2, Rotation2d.kZero) });
    }

    // Accept the swerve drive state and telemeterize it to SmartDashboard.
    public void telemeterize(SwerveDriveState state) {
        var _Timer = new Timer("");

        // Telemeterize the swerve drive state
        stateTable.set("pose", state.Pose);
        stateTable.set("speeds", state.Speeds);
        stateTable.set("module states", state.ModuleStates);
        stateTable.set("module targets", state.ModuleTargets);
        stateTable.set("module positions", state.ModulePositions);
        stateTable.set("timestamp", state.Timestamp);
        stateTable.set("odometry period", state.OdometryPeriod);
        stateTable.set("odometry frequency", 1.0 / state.OdometryPeriod);

        // Also write to log file
        poseArray[0] = state.Pose.getX();
        poseArray[1] = state.Pose.getY();
        poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
            moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
            moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        field.setRobotPose(this.drivetrain.getEstimatedPosition());
        field.getObject("turret_pose").setPose(this.turret.getMechanismPose());

        var modules = this.drivetrain.getModules();
        NTable[] tables = Arrays.stream(new String[] { "FL", "FR", "BL", "BR" })
                .map(s -> moduleTable.sub(s))
                .toArray(NTable[]::new);
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            NTable table = tables[i];
            table.set("steer position", Units.radiansToRotations(
                    MathUtil.angleModulus(module.getCurrentState().angle.getMeasure().in(Radians))));
            table.set("steer setpoint", Units
                    .radiansToRotations(MathUtil.angleModulus(module.getTargetState().angle.getMeasure().in(Radians))));
            table.set("drive velocity", Math.abs(state.ModuleStates[i].speedMetersPerSecond));
            table.set("drive voltage", module.getDriveMotor().getMotorVoltage().getValueAsDouble());
            table.set("steer voltage", module.getSteerMotor().getMotorVoltage().getValueAsDouble());

            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond
                    / (2 * CompbotTunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond)));

            table.set("mechanism", moduleMechanisms[i]);
        }
        _Timer.toc();
    }
}
