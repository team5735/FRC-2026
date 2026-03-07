package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.TurretConstants.FORWARD_LIMIT_BOT_REL;
import static frc.robot.constants.TurretConstants.FORWARD_LIMIT_TUR_REL;
import static frc.robot.constants.TurretConstants.KA;
import static frc.robot.constants.TurretConstants.KD;
import static frc.robot.constants.TurretConstants.KI;
import static frc.robot.constants.TurretConstants.KP;
import static frc.robot.constants.TurretConstants.KS;
import static frc.robot.constants.TurretConstants.KV;
import static frc.robot.constants.TurretConstants.MAX_ACC;
import static frc.robot.constants.TurretConstants.MAX_VEL;
import static frc.robot.constants.TurretConstants.REVERSE_LIMIT_TUR_REL;
import static frc.robot.constants.TurretConstants.SOFT_PADDING;
import static frc.robot.constants.TurretConstants.START_POS_BOT_REL;
import static frc.robot.constants.TurretConstants.formatInputRobotRel;
import static frc.robot.constants.TurretConstants.robotRelToTurretRel;
import static frc.robot.constants.TurretConstants.turretRelToRobotRel;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import frc.robot.SingleSubsystem;
import frc.robot.constants.Constants;
import frc.robot.constants.robot.CompbotConstants;
import frc.robot.constants.robot.RobotConstants;
import frc.robot.util.TunableProfiledPIDController;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX kraken = new TalonFX(Constants.TURRET_MOTOR_ID);
    private final DigitalInput hallLimit = new DigitalInput(Constants.TURRET_LIMIT_PIN);

    public final Trigger limitTrigger = new Trigger(() -> !hallLimit.get());
    public boolean isZeroed = false;

    private final TunableProfiledPIDController pid = new TunableProfiledPIDController("turret", KP, KI, KD,
            MAX_VEL.in(RotationsPerSecond), MAX_ACC.in(RotationsPerSecondPerSecond));
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(KS, KV, KA);
    
    private Supplier<Pose2d> robotPoseSupplier;
    private double prevVel = 0;
    private RobotConstants driveConstants;

    public TurretSubsystem(Supplier<Pose2d> robotPoseSupplier, RobotConstants driveConstants) {
        this.driveConstants = driveConstants;
        kraken.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive));
        kraken.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(10));
        resetAngle(START_POS_BOT_REL);
        pid.setup(robotRelToTurretRel(START_POS_BOT_REL).in(Rotations));
        pid.reset(robotRelToTurretRel(START_POS_BOT_REL).in(Rotations));

        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("turret/posRots", getAngle().in(Rotations));
        SmartDashboard.putNumber("turret/velRPS", kraken.getVelocity().getValue().in(RotationsPerSecond));
        SmartDashboard.putNumber("turret/setpointPosRots",
                turretRelToRobotRel(Rotations.of(pid.getController().getSetpoint().position)).in(Rotations));
        SmartDashboard.putNumber("turret/setpointVelRPS", pid.getController().getSetpoint().velocity);
        SmartDashboard.putNumber("turret/volts", kraken.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("turret/posErrorRots", pid.getController().getPositionError());
        SmartDashboard.putBoolean("turret/limitEngaged", !hallLimit.get());
        SmartDashboard.putBoolean("turret/isZeroed", isZeroed);
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
                        .angularPosition(getAngleTurretRel());
            }, this));
    public final BooleanSupplier isAtForwardLim = () -> {
        return getAngleTurretRel().gte(FORWARD_LIMIT_TUR_REL.minus(SOFT_PADDING));
    };
    public final BooleanSupplier isAtReverseLim = () -> {
        return getAngleTurretRel().lte(REVERSE_LIMIT_TUR_REL.plus(SOFT_PADDING));
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
                .andThen(routine.dynamic(Direction.kForward).until(isAtForwardLim))
                .andThen(routine.dynamic(Direction.kReverse).until(isAtReverseLim))
                .andThen(routine.quasistatic(Direction.kForward).until(isAtForwardLim))
                .andThen(routine.quasistatic(Direction.kReverse).until(isAtReverseLim))
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
    private Command trackStateTurretRel(Supplier<State> goalSupplier) {
        return startRun(
                () -> pid.reset(new State(getAngleTurretRel().in(Rotations),
                        kraken.getVelocity().getValue().in(RotationsPerSecond))),
                () -> {
                    double newVel = pid.getController().getSetpoint().velocity;
                    double voltsToSet = pid.calculate(getAngleTurretRel().in(Rotations), goalSupplier.get())
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
                    pid.reset(new State(getAngleTurretRel().in(Rotations),
                            kraken.getVelocity().getValue().in(RotationsPerSecond)));
                    pid.setGoal(new State(formatInputRobotRel(goal).in(Rotations), 0));
                },
                () -> {
                    double newVel = pid.getController().getSetpoint().velocity;
                    double voltsToSet = pid.calculate(getAngleTurretRel().in(Rotations))
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
        return trackStateTurretRel(() -> new State(formatInputRobotRel(angleSupplier.get()).in(Rotations), 0));
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
        return trackStateTurretRel(() -> new State(formatInputRobotRel(angleSupplier.get()).in(Rotations),
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
        return trackRobotRel(
                () -> formatInputRobotRel(fieldAngle.minus(robotPoseSupplier.get().getRotation().getMeasure())));
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
     * Testing method that resets the Turret's {@link TunableProfiledPIDController}
     * to the constants set in NT
     */
    public void remakePID() {
        State currentState = new State(getAngleTurretRel().in(Rotations), 0);
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
        return turretRelToRobotRel(getAngleTurretRel());
    }

    /**
     * @return the turret-relative {@link Angle} of the turret.
     *         <p>
     *         Turret-relative is
     *         an offset position space, where zero is halfway between the turrets
     *         limits; it is only used for internal control logic.
     */
    public Angle getAngleTurretRel() {
        return kraken.getPosition().getValue();
    }

    /**
     * Sets the known angle of the turret subsystem to a new value
     * 
     * @param newPos the new robot-relative {@link Angle}
     */
    private void resetAngle(Angle newPos) {
        kraken.setPosition(robotRelToTurretRel(newPos));
    }

    /**
     * @return the field-relative {@link Pose2d} of the turret's center, offset from
     *         the robot's position, with its rotational heading being the angle of
     *         the turret
     */
    public Pose2d getMechanismPose() {
        Pose2d robotPoseInField = robotPoseSupplier.get();
        return new Pose2d(driveConstants.getRobotToTurretCenter().rotateBy(robotPoseInField.getRotation())
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
        return testForward().until(limitTrigger::getAsBoolean).andThen(zeroCommand())
                .andThen(holdRobotRel(FORWARD_LIMIT_BOT_REL.minus(SOFT_PADDING)));
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
            resetAngle(FORWARD_LIMIT_BOT_REL);
            isZeroed = true;
        }).ignoringDisable(true);
    }

    public static class Tester extends SingleSubsystem {
        private final TurretSubsystem turret = new TurretSubsystem(() -> Pose2d.kZero, new CompbotConstants());

        public Tester() {
            super();
            // turret.setDefaultCommand(turret.holdRobotRel(Rotations.of(0)));
            turret.limitTrigger.onTrue(turret.zeroCommand()); // resets the turrets position when it engages the
                                                              // Hall-Effect
                                                              // sensor
            turret.setDefaultCommand(turret.holdRobotRel(Rotations.of(0)));

            controller.a().onTrue(turret.holdRobotRel(Rotations.of(0.25)));
            controller.b().onTrue(turret.holdRobotRel(Rotations.of(0.75)));
            controller.rightBumper().whileTrue(turret.trackRobotRel(() -> {
                double x = controller.getRightX();
                double y = controller.getRightY();
                return new Rotation2d(-y, -x).getMeasure();
            }));
            controller.x().whileTrue(turret.zeroSequence());

            controller.povUp().whileTrue(turret.sysId());
            controller.povDown().onTrue(Commands.runOnce(turret::remakePID, turret));
        }
    }
}