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
    private final TalonFX intakeRoller = new TalonFX(Constants.INTAKE_ROLLER_TALONFX_ID);
    private final DutyCycleOut rollerRequest = new DutyCycleOut(0);
    private final DutyCycleOut slapdownRequest = new DutyCycleOut(0);
    private final TunablePIDController pidController = new TunablePIDController("Intake Up and Down");

    public IntakeSubsystem() {
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeRoller.getConfigurator().apply(rollerConfig);

        TalonFXConfiguration slapdownConfig = new TalonFXConfiguration();
        slapdownConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intake_slapdown.getConfigurator().apply(slapdownConfig);
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
        intakeRoller.setControl(rollerRequest.withOutput(1));
    }

    public void reverse() {
        intakeRoller.setControl(rollerRequest.withOutput(-1));
    }

    public void stop() {
        intakeRoller.setControl(rollerRequest.withOutput(0));
    }

    public void setGoal(double goal) {
        pidController.setup(goal);
    }

    public void usePID() {
        double pos = intakeRoller.getPosition().getValueAsDouble();
        double output = pidController.calculate(pos);
        intake_slapdown.setControl(slapdownRequest.withOutput(output));
    }
}
