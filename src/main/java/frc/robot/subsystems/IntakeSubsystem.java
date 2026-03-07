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
import frc.robot.SingleSubsystem;
import frc.robot.constants.Constants;
import frc.robot.util.TunablePIDController;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeSlapdown = new TalonFX(Constants.INTAKE_SLAPDOWN_TALONFX_ID);
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
        intakeSlapdown.getConfigurator().apply(slapdownConfig);
    }

    public void forwardRoll() {
        intakeRoller.setVoltage(4);
    }

    public void reverseRoll() {
        intakeRoller.setVoltage(-4);
    }

    public void stopRoll() {
        intakeRoller.setVoltage(0);
    }

    public void setSlapdownGoal(double goal) {
        pidController.setup(goal);
    }

    public void useSlapdownPID() {
        double pos = intakeSlapdown.getPosition().getValueAsDouble();
        double output = pidController.calculate(pos);
        intakeSlapdown.setControl(slapdownRequest.withOutput(output));
    }

    public Command getIntakeForwardRollCommand() {
        return startEnd(() -> forwardRoll(), () -> stopRoll());
    }

    public Command getIntakeReverseRollCommand() {
        return startEnd(() -> reverseRoll(), () -> stopRoll());
    }

    public Command getLiftCommand(double targetPosition) {
        return startRun(() -> setSlapdownGoal(targetPosition), () -> useSlapdownPID()).finallyDo(a -> stopRoll());
    }

    public Command getSlapdownCommand(double targetPosition) {
        return startRun(() -> setSlapdownGoal(targetPosition), () -> useSlapdownPID()).finallyDo(a -> stopRoll());
    }

    public static class Tester extends SingleSubsystem {
        private final IntakeSubsystem intake = new IntakeSubsystem();

        public Tester() {
            super();

            controller.a().whileTrue(intake.getIntakeForwardRollCommand());
            controller.b().whileTrue(intake.getIntakeReverseRollCommand());
        }
    }
}
