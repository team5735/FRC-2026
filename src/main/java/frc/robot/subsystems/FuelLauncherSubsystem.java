// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;


public class FuelLauncherSubsystem extends SubsystemBase {
    // private final TalonFX krakenMotor = new TalonFX(Constants.LAUNCHER_KRAKEN_ID);
    // private final SparkFlex vortexLeft = new SparkFlex(Constants.LAUNCHER_LEFT_VORTEX_ID, MotorType.kBrushless);
    private final SparkFlex vortexRight = new SparkFlex(Constants.LAUNCHER_RIGHT_VORTEX_ID, MotorType.kBrushless);

    private final PIDController PIDController = new PIDController(0, 0, 0);



    /** Creates a new ExampleSubsystem. */
    public FuelLauncherSubsystem() {
        SmartDashboard.putNumber("shooter_volts", Constants.LAUNCHER_VOLTS);
        // vortexLeft.configure(new SparkFlexConfig().follow(Constants.LAUNCHER_RIGHT_VORTEX_ID, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        vortexRight.configure(new SparkFlexConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void activateVoltage() {
        // krakenMotor.setVoltage(SmartDashboard.getNumber("shooter_volts", Constants.LAUNCHER_VOLTS));
        vortexRight.setVoltage(SmartDashboard.getNumber("shooter_volts", Constants.LAUNCHER_VOLTS));
    }

    public void deactivateVoltage() {
        // krakenMotor.setVoltage(0);
        vortexRight.setVoltage(0);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("speed_rpm_raw", vortexRight.getEncoder().getVelocity());
        // SmartDashboard.putNumber("speed_rpm_raw", krakenMotor.getVelocity().getValueAsDouble());
    }
}
