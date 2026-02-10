package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.TurretConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Constants;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import edu.wpi.first.math.system.plant.DCMotor;
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
    
    private final PivotConfig mechanismConfig = new PivotConfig()
        .withHardLimit(LOWER_LIMIT, UPPER_LIMIT)
        .withSmartMotorController(krakenController)
        .withTelemetry("turret", TelemetryVerbosity.HIGH)
        .withStartingPosition(Rotations.of(0))
        .withMechanismPositionConfig(new MechanismPositionConfig().withRelativePosition())
        

    private final Pivot turretMechanism = new Pivot(mechanismConfig);

    public TurretSubsystem() {
    }

    private void testForward() {
        kraken.setVoltage(TESTING_VOLTS);
    }

    private void testReverse() {
        kraken.setVoltage(-TESTING_VOLTS);
    }

    private void stop() {
        kraken.setVoltage(0);
    }

    public Command testForwardCommand() {
        return runEnd(this::testForward, this::stop);
    }

    public Command testReverseCommand() {
        return runEnd(this::testReverse, this::stop);
    }
}
