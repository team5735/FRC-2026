package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.TurretConstants.KA;
import static frc.robot.constants.TurretConstants.KD;
import static frc.robot.constants.TurretConstants.KI;
import static frc.robot.constants.TurretConstants.KP;
import static frc.robot.constants.TurretConstants.KS;
import static frc.robot.constants.TurretConstants.KV;
import static frc.robot.constants.TurretConstants.LOWER_LIMIT;
import static frc.robot.constants.TurretConstants.MAX_ACC;
import static frc.robot.constants.TurretConstants.MAX_VEL;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotConstants;
import frc.robot.util.TunableProfiledPIDController;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX kraken = new TalonFX(Constants.TURRET_MOTOR_ID);
    private final DigitalInput hallLimit = new DigitalInput(Constants.TURRET_LIMIT_PIN);

    public final Trigger limitTrigger = new Trigger(hallLimit::get);

    private final TunableProfiledPIDController pid = new TunableProfiledPIDController("turret", KP, KI, KD,
            MAX_VEL.in(RotationsPerSecond), MAX_ACC.in(RotationsPerSecondPerSecond));
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(KS, KV, KA);

    private Supplier<Pose2d> robotPoseSupplier;
    private double prevVel = 0;
    public TurretSubsystem(Supplier<Pose2d> robotPoseSupplier) {
        kraken.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive));
        kraken.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(10));
        resetAngle(START_POS);
        pid.setup(START_POS.in(Rotations));
        pid.reset(START_POS.in(Rotations));

        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("turret/posRots", kraken.getPosition().getValue().in(Rotations));
        SmartDashboard.putNumber("turret/velRPS", kraken.getVelocity().getValue().in(RotationsPerSecond));
        SmartDashboard.putNumber("turret/setpointPosRots", pid.getController().getSetpoint().position);
        SmartDashboard.putNumber("turret/setpointVelRPS", pid.getController().getSetpoint().velocity);
        SmartDashboard.putNumber("turret/volts", kraken.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("turret/posErrorRots", pid.getController().getPositionError());
    }

    /**
     * Tests the turret's turning capability at a constant voltage
     * 
     * @return {@link Command} that sets the motor to a constant forward voltage on
     *         scheduling and stops it on ending
     */
    public Command testForward() {
        return startEnd(() -> kraken.setVoltage(0.5), () -> kraken.setVoltage(0));
    }

    /**
     * Tests the turret's turning capability at a constant voltage
     * 
     * @return {@link Command} that sets the motor to a constant backward voltage on
     *         scheduling and stops it on ending
     */
    public Command testReverse() {
        return startEnd(() -> kraken.setVoltage(-0.5), () -> kraken.setVoltage(0));
    }

    /**
     * Stops the turret motor completely
     * 
     * @return {@link Command} that sets the motor to a voltage of zero on
     *         scheduling and does not deschedule itself unless interrupted
     */
    public Command stop() {
        return startRun(() -> kraken.setVoltage(0), () -> {
        });
    }

    private SysIdRoutine routine = new SysIdRoutine(
            new Config(Volts.of(0.15).per(Second), Volts.of(1), null, null),
            new Mechanism(v -> kraken.setVoltage(v.in(Volts)), log -> {
                log.motor("turret_motor")
                        .voltage(kraken.getMotorVoltage().getValue())
                        .angularVelocity(kraken.getVelocity().getValue())
                        .angularPosition(kraken.getPosition().getValue());
            }, this));
    public final BooleanSupplier isAtMax = () -> {
        return kraken.getPosition().getValue().gte(UPPER_LIMIT.minus(SOFT_PADDING));
    };
    public final BooleanSupplier isAtMin = () -> {
        return kraken.getPosition().getValue().lte(UPPER_LIMIT.minus(SOFT_PADDING));
    };

    /**
     * SysId command for this subsystem
     * 
     * @return {@link Command} that runs a {@link SysIdRoutine} for this subsystem,
     *         forward and backward, dynamic and quasistatic. All parts of the
     *         routine end when they hit the upper and lower limit of this
     *         subsystem.
     */
    public Command sysId() {
        return Commands.print("Starting Turret SysId")
                .andThen(routine.dynamic(Direction.kForward)/* .until(isAtMax) */)
                .andThen(routine.dynamic(Direction.kReverse)/* .until(isAtMin) */)
                .andThen(routine.quasistatic(Direction.kForward)/* .until(isAtMax) */)
                .andThen(routine.quasistatic(Direction.kReverse)/* .until(isAtMin) */)
                .andThen(Commands.print("SysId End"));
    }

    /**
     * Runs the turret to track a specified, moving {@link State}. Private because
     * the
     * positional aspect is not clamped. Use {@link #trackRobotRelWithVelocity()}
     * instead.
     * 
     * @param goalSupplier Combined position/velocity supplier in the form of a
     *                     {@link State}.
     * @return {@link Command} that repeatedly applies the output of the
     *         {@link ProfiledPIDController} to the motor.
     */
    private Command trackStateRobotRel(Supplier<State> goalSupplier) {
        return startRun(
                () -> pid.reset(new State(kraken.getPosition().getValue().in(Rotations),
                        kraken.getVelocity().getValue().in(RotationsPerSecond))),
                () -> {
                    double newVel = pid.getController().getSetpoint().velocity;
                    double voltsToSet = pid.calculate(kraken.getPosition().getValue().in(Rotations), goalSupplier.get())
                            + ff.calculateWithVelocities(newVel, newVel
                                    + 0.02 * MAX_ACC.in(RotationsPerSecondPerSecond) * Math.signum(newVel - prevVel));
                    kraken.setVoltage(voltsToSet);
                    prevVel = newVel;
                });
    }

    /**
     * Runs the turret to a specified, moving {@link Angle}.
     * 
     * @param goal Robot-relative {@link Angle}, will be
     *             clamped to the functional range of this subsystem
     *             automatically.
     * @return {@link Command} that sets the {@link ProfiledPIDController}'s goal to
     *         the parameter, then repeatedly applies its output to the motor.
     */
    public Command holdRobotRel(Angle goal) {
        return startRun(
                () -> {
                    pid.reset(new State(kraken.getPosition().getValue().in(Rotations),
                            kraken.getVelocity().getValue().in(RotationsPerSecond)));
                    pid.setGoal(new State(clampInput(goal).in(Rotations), 0));
                },
                () -> {
                    double newVel = pid.getController().getSetpoint().velocity;
                    double voltsToSet = pid.calculate(kraken.getPosition().getValue().in(Rotations))
                            + ff.calculateWithVelocities(newVel, newVel
                                    + 0.02 * MAX_ACC.in(RotationsPerSecondPerSecond) * Math.signum(newVel - prevVel));
                    kraken.setVoltage(voltsToSet);
                    prevVel = newVel;
                });
    }

    /**
     * Runs the turret to a specified, moving {@link Angle}.
     * 
     * @param angleSupplier Supplier for a robot-relative {@link Angle}, will be
     *                      clamped to the functional range of this subsystem
     *                      automatically.
     * @return {@link Command} that repeatedly applies the output of the
     *         {@link ProfiledPIDController} to the motor.
     */
    public Command trackRobotRel(Supplier<Angle> angleSupplier) {
        return trackStateRobotRel(() -> new State(clampInput(angleSupplier.get()).in(Rotations), 0));
    }

    /**
     * Runs the turret to a specified, moving {@link Angle}, with a goal velocity at
     * said angle.
     * 
     * @param angleSupplier   supplier for a robot-relative {@link Angle}, will be
     *                        clamped to the functional range of this subsystem
     *                        automatically.
     * @param velocitySuppler supplier for a robot-relative {@link AngularVelocity}
     *                        that should be reached at the goal position
     * @return {@link Command} that repeatedly applies the output of the
     *         {@link ProfiledPIDController} to the motor.
     */
    public Command trackRobotRelWithVelocity(Supplier<Angle> angleSupplier,
            Supplier<AngularVelocity> velocitySupplier) {
        return trackStateRobotRel(() -> new State(clampInput(angleSupplier.get()).in(Rotations),
                velocitySupplier.get().in(RotationsPerSecond)));
    }

    /**
     * Runs the turret to a specified, static {@link Angle}.
     * 
     * @param fieldAngle Field-relative {@link Angle}.
     * 
     * @return {@link Command} that repeatedly applies the output of the
     *         {@link ProfiledPIDController} to the motor.
     */
    public Command holdFieldRelative(Angle fieldAngle) {
        return trackRobotRel(() -> clampInput(fieldAngle.minus(robotPoseSupplier.get().getRotation().getMeasure())));
    }

    /**
     * Runs the turret to aim at a specified, static {@link Translation2d}.
     * 
     * @param positionToTrack Field-relative {@link Translation2d}.
     * @return {@link Command} that repeatedly applies the output of the
     *         {@link ProfiledPIDController} to the motor.
     */

    public Command trackFieldPos(Translation2d positionToTrack) {
        return trackRobotRel(() -> {
            Pose2d robotPoseInField = robotPoseSupplier.get();
            Translation2d mechanismInField = getMechanismPose().getTranslation();
            Angle fieldAngle = positionToTrack.minus(mechanismInField).getAngle().getMeasure();
            return fieldAngle.minus(robotPoseInField.getRotation().getMeasure());
        });
    }

    /**
     * Clamps a goal angle to the functional range of this subsystem.
     * 
     * @param input the goal position to be clamped.
     * 
     * @return {@link Angle} inclusively between the upper and lower bounds of this
     *         subsystem, either the rotational equivalent of the input, or one of
     *         the bounds.
     */
    public Angle clampInput(Angle input) {
        double softLowerLimit = MathUtil.inputModulus(LOWER_LIMIT.plus(SOFT_PADDING).in(Rotations), 0, 1);
        double softUpperLimit = MathUtil.inputModulus(UPPER_LIMIT.minus(SOFT_PADDING).in(Rotations), 0, 1);
        double moddedInput = MathUtil.inputModulus(input.in(Rotations), 0, 1);
        return Rotations.of(MathUtil.clamp(moddedInput, softLowerLimit, softUpperLimit));
    }

    /**
     * Testing method that resets the Turret's {@link TunableProfiledPIDController}
     * to the constants set in NT
     */
    public void remakePID() {
        State currentState = new State(getAngle().in(Rotations), 0);
        pid.setup(currentState, 0);
        pid.reset(currentState);
    }

    /**
     * @return the {@link Rotation2d} representing the robot-relative angle of the
     *         turret
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(getAngle().in(Rotations));
    }

    /**
     * @return the robot-relative {@link Angle} of the turret
     */
    public Angle getAngle() {
        return kraken.getPosition().getValue();
    }

    /**
     * Sets the known angle of the turret subsystem to a new value
     * 
     * @param newPos the new robot-relative {@link Angle}
     */
    private void resetAngle(Angle newPos) {
        kraken.setPosition(clampInput(newPos));
    }

    /**
     * @return the field-relative {@link Pose2d} of the turret's center, offset from
     *         the robot's position, with its rotational heading being the angle of
     *         the turret
     */
    public Pose2d getMechanismPose() {
        Pose2d robotPoseInField = robotPoseSupplier.get();
        return new Pose2d(RobotConstants.ROBOT_TO_TURRET_CENTER.rotateBy(robotPoseInField.getRotation())
                .plus(robotPoseInField.getTranslation()), getRotation().plus(robotPoseInField.getRotation()));
    }

    /**
     * {@link Command} to zero (find the definite position of) this subsystem. WIll
     * run the motor until the turret reaches its limit switch, then will reset its
     * position, and will finally set its control to hold an angle very close to
     * said limit.
     * 
     * @return Command that performs the aforementioned task
     */
    public Command zeroSequence() {
        // TODO - determine if limit switch is upper/forward or lowe/backwards and
        // change accordingly
        return testForward().until(hallLimit::get).andThen(zeroCommand())
                .andThen(holdRobotRel(UPPER_LIMIT.minus(SOFT_PADDING)));
    }

    /**
     * Non-requiring {@link Command} that simply zeroes the position of this
     * subsytem to that of its Hall-Effect limit switch being engaged
     * <p>
     * This is intended to be bound to {@link TurretSubsystem#limitTrigger} and used
     * by very little else.
     * 
     * @return a Command generated with {@link Commands#runOnce()} that sets this
     *         subsystem's known angle to its limit
     */
    public Command zeroCommand() {
        return Commands.runOnce(() -> {
            resetAngle(UPPER_LIMIT);
        });
    }
}
