package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class SpinDexSubsystem extends SubsystemBase {

    public final SparkFlex talon6 = new SparkFlex(Constants.motor6, MotorType.kBrushless);
    public final SparkFlex talon27 = new SparkFlex(Constants.motor27, MotorType.kBrushless);

    public SpinDexSubsystem() {
        talon6.clearFaults();
        talon27.clearFaults();
    }

    public void Motor6Run() {
        talon6.setVoltage(-5);
    }

    public void Motor6stop() {
        talon6.setVoltage(0);
    }

    public void Motor27Run() {
        talon27.setVoltage(-3);
    }

    public void Motor27stop() {
        talon27.setVoltage(0);
    }
}
