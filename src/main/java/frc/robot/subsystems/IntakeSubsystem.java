// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SingleSubsystem;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeSlapdown = new TalonFX(Constants.INTAKE_SLAPDOWN_TALONFX_ID);
    private final TalonFX intakeRoller = new TalonFX(Constants.INTAKE_ROLLER_TALONFX_ID);
    private final ArmFeedforward ff = new ArmFeedforward(0, 0, 0);
    private final DigitalInput hallLimit = new DigitalInput(Constants.INTAKE_LIMIT_PIN);

    public IntakeSubsystem() {
        super();
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeRoller.getConfigurator().apply(rollerConfig);

        TalonFXConfiguration slapdownConfig = new TalonFXConfiguration();
        slapdownConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        slapdownConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        slapdownConfig.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_REDUCTION;
        intakeSlapdown.getConfigurator().apply(slapdownConfig);
    }

    private void forwardRoll() {
        intakeRoller.setVoltage(IntakeConstants.ROLL_IN_VOLTS);
    }

    private void reverseRoll() {
        intakeRoller.setVoltage(IntakeConstants.ROLL_OUT_VOLTS);
    }

    private void stopRoll() {
        intakeRoller.setVoltage(0);
    }

    private void runSlapdown(AngularVelocity velocity){
        intakeSlapdown.setVoltage(ff.calculate(getSlapdownPosition().in(Rotations), velocity.in(RotationsPerSecond)));
    }

    public boolean isAtPosition(Angle position){
        return getSlapdownPosition().isNear(position, IntakeConstants.ANGULAR_TOLERANCE);
    }

    public Angle getSlapdownPosition(){
        return intakeSlapdown.getPosition().getValue();
    }

    public Command getIntakeForwardRollCommand() {
        return startEnd(() -> forwardRoll(), () -> stopRoll());
    }

    public Command getIntakeReverseRollCommand() {
        return startEnd(() -> reverseRoll(), () -> stopRoll());
    }

    public Command getLiftCommand() {
        return runEnd(() -> runSlapdown(IntakeConstants.LIFT_VEL), () -> intakeSlapdown.setVoltage(0)).until(() ->  isAtPosition(IntakeConstants.UPPER_RELEASE_POS));
    }

    public Command getSlapdownCommand() {
        return runEnd(() -> runSlapdown(IntakeConstants.SLAPDOWN_VEL.unaryMinus()), () -> intakeSlapdown.setVoltage(0)).until(() ->  isAtPosition(IntakeConstants.UPPER_RELEASE_POS));
    }

    public Command zeroSlapdownPosition(){
        return Commands.runOnce(() -> intakeSlapdown.setPosition(IntakeConstants.START_POS)).ignoringDisable(true);
    }

    public Trigger limitEngaged = new Trigger(() -> !hallLimit.get());

    public static class Tester extends SingleSubsystem {
        private final IntakeSubsystem intake = new IntakeSubsystem();

        public Tester() {
            super();
            intake.limitEngaged.onTrue(intake.zeroSlapdownPosition());

            controller.a().whileTrue(intake.getIntakeForwardRollCommand());
            controller.b().whileTrue(intake.getIntakeReverseRollCommand());
        }
    }
}
