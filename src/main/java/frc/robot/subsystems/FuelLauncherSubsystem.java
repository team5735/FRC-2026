// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FuelLauncherSubsystem extends SubsystemBase {
  private final TalonFX krakenMotor = new TalonFX(Constants.LAUNCHER_KRAKEN_ID);
  /** Creates a new ExampleSubsystem. */
  public FuelLauncherSubsystem() {}

  public void ActivateVoltage() {
    krakenMotor.setVoltage(Constants.LAUNCHER_VOLTS);
  }

  public void DeactivateVoltage() {
    krakenMotor.setVoltage(0);
  }
}
