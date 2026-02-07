package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX kraken = new TalonFX(Constants.TURRET_MOTOR_ID);

    public TurretSubsystem() {
        kraken.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    private void testForward() {
        kraken.setVoltage(TurretConstants.TESTING_VOLTS);
    }

    private void testReverse() {
        kraken.setVoltage(-TurretConstants.TESTING_VOLTS);
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
