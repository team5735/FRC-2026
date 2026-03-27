package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PartialRobot;
import frc.robot.constants.Constants;
import frc.robot.util.NTable;

public class SpinDexSubsystem extends SubsystemBase {
    public final SparkFlex feedVortex = new SparkFlex(Constants.SPINDEX_FEED_VORTEX_ID, MotorType.kBrushless);
    public final SparkFlex wheelVortex = new SparkFlex(Constants.SPINDEX_WHEEL_VORTEX_ID, MotorType.kBrushless);
    private NTable table = NTable.root("tuning").sub("spindex");

    public SpinDexSubsystem() {
        super();

        feedVortex.clearFaults();
        wheelVortex.clearFaults();

        // ensure these values are in NT and persistent
        // they can be changed for tuning purposes, but...
        table.ensure("feed", -6);
        table.ensure("wheel: fwd", -4);
        table.ensure("wheel: bck", 5);

        // they should default to these values on robot start
        table.set("feed", -6);
        table.set("wheel: fwd", -4);
        table.set("wheel: bck", 5);
    }

    public void runFeeder() {
        feedVortex.setVoltage(table.getDouble("feed"));
    }

    public void reverseFeeder() {
        feedVortex.setVoltage(-table.getDouble("feed"));
    }

    public void stopFeeder() {
        feedVortex.setVoltage(0);
    }

    public double getForwardVoltage(){
        return table.getDouble("wheel: fwd");
    }

    public void runWheel() {
        wheelVortex.setVoltage(table.getDouble("wheel: fwd"));
    }

    public void stopWheel() {
        wheelVortex.setVoltage(0);
    }

    public void reverseWheel() {
        wheelVortex.setVoltage(table.getDouble("wheel: bck"));
    }

    public Command getRun() {
        return startEnd(() -> {
            runWheel();
            runFeeder();
        }, () -> {
            stopWheel();
            stopFeeder();
        });
    }

    public Command getRunSupplier(Supplier<Double> wheel, Supplier<Double> feeder) {
        return startEnd(() -> {
            wheelVortex.setVoltage(wheel.get());
            feedVortex.setVoltage(feeder.get());
        }, () -> {
            stopWheel();
            stopFeeder();
        });
    }

    public Command getInformedRun(BooleanSupplier isValid) {
        return runEnd(() -> {
            if (isValid.getAsBoolean()) {
                runWheel();
                runFeeder();
            } else {
                stopWheel();
                stopFeeder();
            }
        }, () -> {
            stopWheel();
            stopFeeder();
        });
    }

    public Command getBackwards() {
        return startEnd(() -> {
            reverseWheel();
            reverseFeeder();
        }, () -> {
            stopWheel();
            stopFeeder();
        });
    }

    public static class Tester extends PartialRobot {
        private final SpinDexSubsystem spindex = new SpinDexSubsystem();

        public Tester() {
            controller.a().whileTrue(spindex.getRun());

        }
    }
}
