// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.FuelLauncherConstants;
import frc.robot.util.TunablePIDController;


public class FuelLauncherSubsystem extends SubsystemBase {
    private final TalonFX krakenLeft = new TalonFX(Constants.LAUNCHER_LEFT_KRAKEN_ID);
    private final TalonFX krakenRight = new TalonFX(Constants.LAUNCHER_RIGHT_KRAKEN_ID);

    private final TunablePIDController PIDController = new TunablePIDController("fuel_launcher", 0, 0, 0);



    /** Creates a new ExampleSubsystem. */
    public FuelLauncherSubsystem() {
        SmartDashboard.putNumber("shooter_volts", Constants.LAUNCHER_VOLTS);
        krakenRight.setControl(new Follower(Constants.LAUNCHER_LEFT_KRAKEN_ID, MotorAlignmentValue.Opposed));
    }

    public double getMotorRPM() {
        return 60*krakenLeft.getVelocity().getValueAsDouble(); 
    }

    public void activateVoltage() {
        krakenLeft.setVoltage(SmartDashboard.getNumber("shooter_volts", Constants.LAUNCHER_VOLTS));
    }

    public void deactivateVoltage() {
        krakenLeft.setVoltage(0);
    }

    public void setTargetRPM(double rpm) {
        PIDController.setup(rpm, FuelLauncherConstants.RPM_TOLERANCE);
    }

    // Multiplying krakenMotor by 60 to turn rotations per second into rotations per minute (RPM)
    public void usePID() {
    double rpm = this.getMotorRPM(); 
    double output = PIDController.calculate(rpm);
    krakenLeft.set(output);
  }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("speed_rpm_raw", 60*krakenLeft.getVelocity().getValueAsDouble());
    }
}
