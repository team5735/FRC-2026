package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class SpinDexSubsystem extends SubsystemBase {
    public final SparkFlex feedVortex = new SparkFlex(Constants.SPINDEX_FEED_VORTEX_ID, MotorType.kBrushless);
    public final SparkFlex wheelVortex = new SparkFlex(Constants.SPINDEX_WHEEL_VORTEX_ID, MotorType.kBrushless);

    public SpinDexSubsystem() {
        feedVortex.clearFaults();
        wheelVortex.clearFaults();
    }

    public void runFeeder() {
        feedVortex.setVoltage(-5);
    }

    public void stopFeeder() {
        feedVortex.setVoltage(0);
    }

    public void runWheel() {
        wheelVortex.setVoltage(-2);
    }

    public void stopWheel() {
        wheelVortex.setVoltage(0);
    }
    
    public Command getStart() {
        return startEnd(() -> {
            runWheel();
            runFeeder();
        }, () -> {
            stopWheel();
            stopFeeder();
        });
    }
}
