package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;

public class ClimberSubsystem extends SubsystemBase {
    public final TalonFX kraken = new TalonFX(Constants.CLIMB_KRAKEN_ID);

    private final DigitalInput limitUp = new DigitalInput(Constants.CLIMB_UPPER_LIMIT_PIN);
    private final DigitalInput limitDown = new DigitalInput(Constants.CLIMB_LOWER_LIMIT_PIN);
    private boolean canMoveUp;
    private boolean canMoveDown;

    public ClimberSubsystem() {
        kraken.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    public Command getClimbDownCommand() {
        return run(() -> climbDown()).finallyDo(a -> stop());
    }

    public Command getClimbUpCommand() {
        return run(() -> climbUp()).finallyDo(a -> stop());
    }

    public Command getFullyExtendCommand() {
        return run(() -> climbUp()).until(this::isAtUpLimit).finallyDo(() -> stop());
    }

    public Command getFullyDetractCommand() {
        return run(() -> climbDown()).until(this::isAtDownLimit).finallyDo(() -> stop());
    }

    public void climbUp() {
        if (canMoveUp) {
            kraken.setVoltage(ClimberConstants.UP_VOLTS);
        } else {
            kraken.setVoltage(0);
        }
    }

    public void climbDown() {
        if (canMoveDown) {
            kraken.setVoltage(ClimberConstants.DOWN_VOLTS);
        } else {
            kraken.setVoltage(0);
        }
    }

    public void stop() {
        kraken.setVoltage(0);
    }

    public boolean isAtUpLimit() {
        return !limitUp.get();
    }

    public boolean isAtDownLimit() {
        return !limitDown.get();
    }

    @Override
    public void periodic() {
        // DIO
        canMoveUp = limitUp.get();
        canMoveDown = limitDown.get();

        if (!canMoveDown && kraken.getMotorVoltage().getValueAsDouble() < 0) {
            kraken.setVoltage(0);
        } else if (!canMoveUp && kraken.getMotorVoltage().getValueAsDouble() > 0) {
            kraken.setVoltage(0);
        }

        SmartDashboard.putBoolean("limitDown", limitDown.get());
        SmartDashboard.putBoolean("limitUp", limitUp.get());

        double current = kraken.getSupplyCurrent().getValueAsDouble();
        SmartDashboard.putNumber("Climber/Current", current);

        double velocity = kraken.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Climber/Velocity", velocity);

        double forwardLimit = kraken.getForwardLimit().getValueAsDouble();
        SmartDashboard.putNumber("Climber/ForwardLimit", forwardLimit);

        double reverseLimit = kraken.getReverseLimit().getValueAsDouble();
        SmartDashboard.putNumber("Climber/ReverseLimit", reverseLimit);
    }
}
