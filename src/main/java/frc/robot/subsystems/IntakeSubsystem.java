// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intake_slapdown = new TalonFX(Constants.INTAKE_TALONFX_ID);
    private final SparkMax intake_roller = new SparkMax(Constants.INTAKE_SPARKMAX_ID, MotorType.kBrushless);
    private final SparkBaseConfig theConfig = new SparkMaxConfig().inverted(true);
    private final PIDController pidController = new PIDController(0, 0, 0);

    public IntakeSubsystem() {
        intake_roller.configure(theConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command getIntakeRollerCommand(){
        return run(()->run()).finallyDo(a->stop());
    }

    public Command getIntakeReverseCommand(){
        return run(()->reverse()).finallyDo(a->stop());
    }

    public void run() {
        intake_roller.setVoltage(1);
    }

    public void reverse() {
        intake_roller.setVoltage(-1);
    }

    public void stop() {
        intake_roller.setVoltage(0);
    }

    public void setGoal(double goal) {
        pidController.setSetpoint(goal);
    }

    public void usePID() {
        double pos = intake_roller.getEncoder().getPosition();
        double output = pidController.calculate(pos);
        intake_slapdown.set(output);
    }

    @Override
    public void periodic() {
        double kP = SmartDashboard.getNumber("PID kP", 0.1);
        double kI = SmartDashboard.getNumber("PID kI", 0);
        double kD = SmartDashboard.getNumber("PID kD", 0);
        pidController.setPID(kP, kI, kD);
    }
}
