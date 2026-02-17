package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.TurretConstants.CONSTRAINTS;
import static frc.robot.constants.TurretConstants.KA;
import static frc.robot.constants.TurretConstants.KD;
import static frc.robot.constants.TurretConstants.KI;
import static frc.robot.constants.TurretConstants.KP;
import static frc.robot.constants.TurretConstants.KS;
import static frc.robot.constants.TurretConstants.KV;
import static frc.robot.constants.TurretConstants.LOWER_LIMIT;
import static frc.robot.constants.TurretConstants.SOFT_PADDING;
import static frc.robot.constants.TurretConstants.START_POS;
import static frc.robot.constants.TurretConstants.UPPER_LIMIT;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotConstants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX kraken = new TalonFX(Constants.TURRET_MOTOR_ID);

    private final ProfiledPIDController pid = new ProfiledPIDController(KP, KI, KD, CONSTRAINTS);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(KS, KV, KA);

    public TurretSubsystem() {
        kraken.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive));
        kraken.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(10));
        kraken.setPosition(START_POS);
        pid.reset(START_POS.in(Rotations));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("turret/posRots", kraken.getPosition().getValue().in(Rotations));
        SmartDashboard.putNumber("turret/velRPS", kraken.getVelocity().getValue().in(RotationsPerSecond));
        SmartDashboard.putNumber("turret/setpointPosRots", pid.getSetpoint().position);
        SmartDashboard.putNumber("turret/setpointVelRPS", pid.getSetpoint().velocity);
        SmartDashboard.putNumber("turret/volts", kraken.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("turret/posErrorRots", pid.getPositionError());
    }

    public Command testForward() {
        return startEnd(() -> kraken.setVoltage(0.5), () -> kraken.setVoltage(0));
    }

    public Command testReverse() {
        return startEnd(() -> kraken.setVoltage(-0.5), () -> kraken.setVoltage(0));
    }

    public Command stop() {
        return startRun(() -> kraken.setVoltage(0), () -> {
        });
    }

    private SysIdRoutine routine = new SysIdRoutine(
            new Config(Volts.of(0.15).per(Second), Volts.of(0.5), null, null),
            new Mechanism(v -> kraken.setVoltage(v.in(Volts)), log -> {
                log.motor("turret_motor")
                        .voltage(kraken.getMotorVoltage().getValue())
                        .angularVelocity(kraken.getVelocity().getValue())
                        .angularPosition(kraken.getPosition().getValue());
            }, this));
    private BooleanSupplier maxTrigger = () -> {
        return kraken.getPosition().getValue().gte(UPPER_LIMIT.minus(SOFT_PADDING));
    };
    private BooleanSupplier minTrigger = () -> {
        return kraken.getPosition().getValue().lte(UPPER_LIMIT.minus(SOFT_PADDING));
    };

    public Command sysId() {
        return Commands.print("Starting Turret SysId")
                .andThen(routine.dynamic(Direction.kForward).until(maxTrigger))
                .andThen(routine.dynamic(Direction.kReverse).until(minTrigger))
                .andThen(routine.quasistatic(Direction.kForward).until(maxTrigger))
                .andThen(routine.quasistatic(Direction.kReverse).until(minTrigger))
                .andThen(Commands.print("SysId End"));
    }

    private Command trackStateRobotRel(Supplier<State> goalSupplier) {
        return run(
                () -> {
                    double voltsToSet = pid.calculate(kraken.getPosition().getValue().in(Rotations), goalSupplier.get())
                            + ff.calculate(pid.getSetpoint().velocity);
                    kraken.setVoltage(voltsToSet);
                });
    }

    public Command trackRobotRel(Supplier<Angle> angleSupplier) {
        return trackStateRobotRel(() -> new State(clampInput(angleSupplier.get()).in(Rotations), 0));
    }

    public Command holdRobotRel(Angle goal) {
        return startRun(
                () -> pid.setGoal(clampInput(goal).in(Rotations)),
                () -> {
                    double voltsToSet = pid.calculate(kraken.getPosition().getValue().in(Rotations))
                            + ff.calculate(pid.getSetpoint().velocity);
                    kraken.setVoltage(voltsToSet);
                });
    }

    public Command holdFieldRelative(Angle fieldAngle, Supplier<Angle> robotAngleSupplier) {
        return trackRobotRel(() -> clampInput(fieldAngle.minus(robotAngleSupplier.get())));
    }

    public Command trackFieldPos(Translation2d positionToTrack, Supplier<Pose2d> robotPoseSupplier) {
        return trackRobotRel(() -> {
            Pose2d robotPoseInField = robotPoseSupplier.get();
            Translation2d robotToTurret = RobotConstants.ROBOT_TO_TURRET_CENTER.toTranslation2d();
            Translation2d mechanismInField = robotToTurret.rotateBy(robotPoseInField.getRotation())
                    .plus(robotPoseInField.getTranslation());
            Angle fieldAngle = positionToTrack.minus(mechanismInField).getAngle().getMeasure();
            return fieldAngle.minus(robotPoseInField.getRotation().getMeasure());
        });
    }

    /**
     * Clamps a goal angle to the acceptable range of this subsystem
     * 
     * @param input the goal position to be clamped
     * 
     * @return
     */
    public Angle clampInput(Angle input) {
        double softLowerLimit = MathUtil.inputModulus(LOWER_LIMIT.plus(SOFT_PADDING).in(Rotations), 0, 1);
        double softUpperLimit = MathUtil.inputModulus(UPPER_LIMIT.minus(SOFT_PADDING).in(Rotations), 0, 1);
        double moddedInput = MathUtil.inputModulus(input.in(Rotations), 0, 1);
        return Rotations.of(MathUtil.clamp(moddedInput, softLowerLimit, softUpperLimit));
    }
}
