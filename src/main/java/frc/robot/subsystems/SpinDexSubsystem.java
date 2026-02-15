package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class SpinDexSubsystem extends SubsystemBase {

    public final TalonFX talon6 = new TalonFX(Constants.motor6);
    public final TalonFX talon27 = new TalonFX(Constants.motor27);

    public SpinDexSubsystem() {
        talon6.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        talon27.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    public void Motor6Run() {
        talon6.setVoltage(4);
    }

    public void Motor6stop() {
        talon6.setVoltage(0);
    }

    public void Motor27Run() {
        talon6.setVoltage(2);
    }

    public void Motor27stop() {
        talon6.setVoltage(0);
    }
}
