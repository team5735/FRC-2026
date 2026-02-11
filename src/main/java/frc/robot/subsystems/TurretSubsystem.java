package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.TurretConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        .withMotorInverted(false);

    private final SmartMotorController krakenController = new TalonFXWrapper(kraken, DCMotor.getKrakenX44(1), krakenControllerConfig);
    
    private final PivotConfig mechanismConfig = new PivotConfig(krakenController)
        .withHardLimit(LOWER_LIMIT, UPPER_LIMIT)
        .withSoftLimits(LOWER_LIMIT.plus(Rotations.of(0.05)), UPPER_LIMIT.minus(Rotations.of(0.05)))
        .withTelemetry("turret", TelemetryVerbosity.HIGH)
        .withStartingPosition(Rotations.of(0))
        .withMOI(MOI)
        .withMechanismPositionConfig(
            new MechanismPositionConfig().withRelativePosition(RobotConstants.ROBOT_TO_TURRET_CENTER)
            );
        

    private final Pivot turretMechanism = new Pivot(mechanismConfig);

    public TurretSubsystem() {
    }

    @Override
    public void periodic(){
        turretMechanism.updateTelemetry();
    }

    @Override
    public void simulationPeriodic(){
        turretMechanism.simIterate();
    }

    public Command testForward() {
        return turretMechanism.set(0.5);
    }

    public Command testReverse() {
        return turretMechanism.set(-0.5);
    }

    public Command stop(){
        return turretMechanism.set(0);
    }

    public Command sysId(){
        return turretMechanism.sysId(Volts.of(7), Volts.of(0.1).per(Second), null);
    }

    public Command holdRobotRelative(Angle robotAngle){
        return turretMechanism.run(robotAngle);
    }

    public Command trackRobotRelative(Supplier<Angle> robotAngleSupplier){
        return turretMechanism.run(robotAngleSupplier);
    }

    public Command holdFieldRelative(Angle fieldAngle, Supplier<Angle> robotAngleSupplier){
        return turretMechanism.run(() -> {
            return fieldAngle.minus(robotAngleSupplier.get());
        });
    }

    public Command trackFieldPos(Translation2d positionToTrack, Supplier<Pose2d> robotPosSupplier){
        return turretMechanism.run(() -> {
            Pose2d robotPos = robotPosSupplier.get();
            Angle fieldAngle = Radians.of(Math.atan((positionToTrack.getY() - robotPos.getY())/(positionToTrack.getX() - robotPos.getX())));
            return fieldAngle.minus(robotPos.getRotation().getMeasure());
        });
    }
}
