// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.TunablePIDController;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intake_slapdown = new TalonFX(Constants.INTAKE_TALONFX_ID);
    private final TalonFX intake_roller = new TalonFX(Constants.INTAKE_ROLLER_TALONFX_ID);
    private final DutyCycleOut rollerRequest = new DutyCycleOut(0); // Ima be honest idek what this line does but the
                                                                    // code doesnt work without it so if someone can
                                                                    // explain it to be that would be amazing.
    private final TunablePIDController pidController = new TunablePIDController("Intake Up and Down");

    public IntakeSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intake_roller.getConfigurator().apply(config);
    }

    public Command getIntakeRollerCommand() {
        return runEnd(() -> forward(), () -> stop());
    }

    public Command getIntakeReverseCommand() {
        return run(() -> reverse()).finallyDo(a -> stop());
    }

    public Command liftCommand(double targetPosition) {
        return startRun(() -> setGoal(targetPosition), () -> usePID()).finallyDo(a -> stop());
    }

    public Command slapdownCommand(double targetPosition) {
        return startRun(() -> setGoal(targetPosition), () -> usePID()).finallyDo(a -> stop());
    }

    public void forward() {
        intake_roller.setControl(rollerRequest.withOutput(1));
    }

    public void reverse() {
        intake_roller.setControl(rollerRequest.withOutput(-1));
    }

    public void stop() {
        intake_roller.setControl(rollerRequest.withOutput(0));
    }

    public void setGoal(double goal) {
        pidController.setup(goal);
    }

    public void usePID() {
        double pos = intake_roller.getPosition().getValueAsDouble();
        double output = pidController.calculate(pos);
        intake_slapdown.set(output);
    }
}
