package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SingleSubsystem;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;

public class ClimberSubsystem extends SubsystemBase {
    public final TalonFX talon = new TalonFX(Constants.CLIMB_TALON_ID);

    private final DigitalInput limitUp = new DigitalInput(Constants.CLIMB_UPPER_LIMIT_PIN);
    private final DigitalInput limitDown = new DigitalInput(Constants.CLIMB_LOWER_LIMIT_PIN);
    private boolean canMoveUp;
    private boolean canMoveDown;

    public ClimberSubsystem() {
        super();
        talon.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    public Command getClimbDownCommand() {
        return run(() -> climbDown()).finallyDo(a -> stop());
    }

    public Command getClimbUpCommand() {
        return run(() -> climbUp()).finallyDo(a -> stop());
    }

    public Command getClimbDownOverrideCommand() {
        return run(() -> climbDownOverride()).finallyDo(a -> stop());
    }

    public Command getClimbUpOverrideCommand() {
        return run(() -> climbUpOverride()).finallyDo(a -> stop());
    }

    public void climbUp() {
        if (canMoveUp) {
            talon.setVoltage(ClimberConstants.UP_VOLTS);
        } else {
            talon.setVoltage(0);
        }
    }

    public void climbDown() {
        if (canMoveDown) {
            talon.setVoltage(ClimberConstants.DOWN_VOLTS);
        } else {
            talon.setVoltage(0);
        }
    }

    public void climbUpOverride() {
        talon.setVoltage(ClimberConstants.UP_VOLTS);
    }

    public void climbDownOverride() {
        talon.setVoltage(ClimberConstants.DOWN_VOLTS);
    }

    public void stop() {
        talon.setVoltage(0);
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

        if (!canMoveDown && talon.getMotorVoltage().getValueAsDouble() < 0) {
            talon.setVoltage(0);
        } else if (!canMoveUp && talon.getMotorVoltage().getValueAsDouble() > 0) {
            talon.setVoltage(0);
        }

        SmartDashboard.putBoolean("limitDown", limitDown.get());
        SmartDashboard.putBoolean("limitUp", limitUp.get());

        double current = talon.getSupplyCurrent().getValueAsDouble();
        SmartDashboard.putNumber("Climber/Current", current);

        double velocity = talon.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Climber/Velocity", velocity);

        double forwardLimit = talon.getForwardLimit().getValueAsDouble();
        SmartDashboard.putNumber("Climber/ForwardLimit", forwardLimit);

        double reverseLimit = talon.getReverseLimit().getValueAsDouble();
        SmartDashboard.putNumber("Climber/ReverseLimit", reverseLimit);
    }

    public static class Tester extends SingleSubsystem {

        ClimberSubsystem climber = new ClimberSubsystem();

        public Tester() {
            super();
            controller.rightTrigger().whileTrue(climber.getClimbUpCommand());
            controller.leftTrigger().whileTrue(climber.getClimbDownCommand());

        }
    };

}
