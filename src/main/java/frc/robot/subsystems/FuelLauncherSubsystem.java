// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FuelLauncherSubsystem extends SubsystemBase {
    private final TalonFX krakenMotor = new TalonFX(Constants.LAUNCHER_KRAKEN_ID);

    /** Creates a new ExampleSubsystem. */
    public FuelLauncherSubsystem() {
        SmartDashboard.putNumber("shooter_volts", Constants.LAUNCHER_VOLTS);
    }

    public void activateVoltage() {
        krakenMotor.setVoltage(SmartDashboard.getNumber("shooter_volts", Constants.LAUNCHER_VOLTS));
    }

    public void deactivateVoltage() {
        krakenMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("speed_rpm_raw", krakenMotor.getVelocity().getValueAsDouble());
    }
}
