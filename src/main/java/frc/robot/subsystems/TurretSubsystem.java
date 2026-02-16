package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.TurretConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
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
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX kraken = new TalonFX(Constants.TURRET_MOTOR_ID);

    private final SmartMotorControllerConfig krakenControllerConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(EXPO_PID)
            .withSimClosedLoopController(SIM_EXPO_PID)
            .withFeedforward(FF)
            .withTelemetry("turret_motor", TelemetryVerbosity.HIGH)
            .withGearing(GEAR_REDUCTION)
            .withIdleMode(MotorMode.BRAKE)
            .withMotorInverted(true)
            .withStartingPosition(Rotations.of(0.5));

    private final SmartMotorController krakenController = new TalonFXWrapper(kraken, DCMotor.getKrakenX44(1),
            krakenControllerConfig);

    private final PivotConfig mechanismConfig = new PivotConfig(krakenController)
            .withHardLimit(LOWER_LIMIT, UPPER_LIMIT)
            .withSoftLimits(LOWER_LIMIT.plus(SOFT_PADDING), UPPER_LIMIT.minus(SOFT_PADDING))
            .withTelemetry("turret", TelemetryVerbosity.HIGH)
            .withMOI(MOI)
            .withMechanismPositionConfig(
                    new MechanismPositionConfig().withRelativePosition(RobotConstants.ROBOT_TO_TURRET_CENTER));

    private final Pivot turretMechanism = new Pivot(mechanismConfig);

    public TurretSubsystem() {
        krakenController.setEncoderPosition(Rotations.of(0.5));
    }

    @Override
    public void periodic() {
        turretMechanism.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        turretMechanism.simIterate();
    }

    public Command testForward() {
        return turretMechanism.setVoltage(Volts.of(0.5)).finallyDo(() -> krakenController.setVoltage(Volts.of(0)));
    }

    public Command testReverse() {
        return turretMechanism.setVoltage(Volts.of(-0.5)).finallyDo(() -> krakenController.setVoltage(Volts.of(0)));
    }

    public Command stop() {
        return turretMechanism.set(0);
    }

    private SysIdRoutine routine = new SysIdRoutine(
                new Config(Volts.of(0.15).per(Second), Volts.of(0.5), null, null),
                new Mechanism(krakenController::setVoltage, log -> {
                    log.motor("turret_motor")
                            .voltage(krakenController.getVoltage())
                            .angularVelocity(krakenController.getMechanismVelocity())
                            .angularPosition(krakenController.getMechanismPosition());
                }, this));
    private Trigger maxTrigger = turretMechanism.gte(UPPER_LIMIT.minus(SOFT_PADDING));
    private Trigger minTrigger = turretMechanism.lte(LOWER_LIMIT.plus(SOFT_PADDING));

    public Command sysId() {        
        return Commands.print("Starting Turret SysId")
                .andThen(Commands.runOnce(krakenController::stopClosedLoopController))
                .andThen(routine.dynamic(Direction.kForward).until(maxTrigger))
                .andThen(routine.dynamic(Direction.kReverse).until(minTrigger))
                .andThen(routine.quasistatic(Direction.kForward).until(maxTrigger))
                .andThen(routine.quasistatic(Direction.kReverse).until(minTrigger))
                .andThen(Commands.print("SysId End"))
                .finallyDo(krakenController::startClosedLoopController);
    }

    public Command holdRobotRelative(Angle robotAngle) {
        return turretMechanism.run(clampInput(robotAngle));
    }

    public Command trackRobotRelative(Supplier<Angle> robotAngleSupplier) {
        return turretMechanism.run(() -> clampInput(robotAngleSupplier.get()));
    }

    public Command holdFieldRelative(Angle fieldAngle, Supplier<Angle> robotAngleSupplier) {
        return turretMechanism.run(() -> {
            return clampInput(fieldAngle.minus(robotAngleSupplier.get()));
        });
    }

    public Command trackFieldPos(Translation2d positionToTrack, Supplier<Pose2d> robotPoseSupplier) {
        return turretMechanism.run(() -> {
            Pose2d robotPoseInField = robotPoseSupplier.get();
            Translation2d robotToTurret = RobotConstants.ROBOT_TO_TURRET_CENTER.toTranslation2d();
            Translation2d mechanismInField = robotToTurret.rotateBy(robotPoseInField.getRotation())
                    .plus(robotPoseInField.getTranslation());
            Angle fieldAngle = positionToTrack.minus(mechanismInField).getAngle().getMeasure();
            return clampInput(fieldAngle.minus(robotPoseInField.getRotation().getMeasure()));
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
