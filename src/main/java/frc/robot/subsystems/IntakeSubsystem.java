// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intake_up_and_down = new TalonFX(Constants.MOTOR_ID_TALONFX);
  private final SparkMax intake_motor = new SparkMax(Constants.MOTOR_ID_SPARKMAX, MotorType.kBrushless);
  private final SparkBaseConfig theConfig = new SparkMaxConfig().inverted(true);
  private final PIDController pidController = new PIDController(0, 0, 0);
  
  public IntakeSubsystem() {
    intake_motor.configure(theConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intake_run() {
    intake_motor.setVoltage(1);
  }

  public void intake_stop() {
    intake_motor.setVoltage(0);
  }

  public void setGoal(double goal) {
    pidController.setSetpoint(goal);
  }

  public void usePID() {
    double ryan = intake_motor.getEncoder().getPosition();
    double output = pidController.calculate(ryan);
    intake_motor.set(output);
  }

  @Override
  public void periodic() {
    double kP = SmartDashboard.getNumber("PID kP", 0.1);
    double kI = SmartDashboard.getNumber("PID kI", 0);
    double kD = SmartDashboard.getNumber("PID kD", 0);
    pidController.setPID(kP, kI, kD);
  }
}

