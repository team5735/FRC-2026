package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class SpinDexSubsystem extends SubsystemBase {
    public final SparkFlex outerMotor = new SparkFlex(Constants.SPINDEX_OUTER_VORTEX_ID, MotorType.kBrushless);
    public final SparkFlex innerMotor = new SparkFlex(Constants.SPINDEX_INNER_VORTEX_ID, MotorType.kBrushless);

    public SpinDexSubsystem() {
        outerMotor.clearFaults();
        innerMotor.clearFaults();
    }

    public void runOuter() {
        outerMotor.setVoltage(-5);
    }

    public void stopOuter() {
        outerMotor.setVoltage(0);
    }

    public void runInner() {
        innerMotor.setVoltage(-3);
    }

    public void stopInner() {
        innerMotor.setVoltage(0);
    }

    public Command getStart() {
        return runOnce(() -> {
            runInner();
            runOuter();
        }).finallyDo(() -> {
            stopInner();
            stopOuter();
        });
    }
}
