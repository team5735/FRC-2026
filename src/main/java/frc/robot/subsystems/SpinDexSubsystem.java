package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PartialRobot;
import frc.robot.constants.Constants;
import frc.robot.util.NTable;

public class SpinDexSubsystem extends SubsystemBase {
    private final SparkFlex feedVortex = new SparkFlex(Constants.SPINDEX_FEED_VORTEX_ID, MotorType.kBrushless);
    private final SparkFlex wheelVortex = new SparkFlex(Constants.SPINDEX_WHEEL_VORTEX_ID, MotorType.kBrushless);
    private NTable table = NTable.root("tuning").sub("spindex");

    // simulation: two Vortexes, simulated through REV's sim, which only exists in simulation
    private SparkFlexSim feedVortexSim = null;
    private SparkFlexSim wheelVortexSim = null;
    private final DCMotorSim feedMotorSim = makeSimVortex();
    private final DCMotorSim wheelMotorSim = makeSimVortex();
    private double simFeedVolts = 0; // last voltage the feed Vortex applied
    private double simWheelVolts = 0; // last voltage the wheel Vortex applied

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

        if (RobotBase.isSimulation()) {
            DCMotor vortex = DCMotor.getNeoVortex(1);
            feedVortexSim = new SparkFlexSim(feedVortex, vortex);
            wheelVortexSim = new SparkFlexSim(wheelVortex, vortex);
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("spindex/wheel_output", wheelVortex.getAppliedOutput());
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
            } else {
                stopWheel();
            }
        }, () -> {
            stopWheel();
            stopFeeder();
        }).beforeStarting(this::runFeeder);
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

    // ---- simulation

    @Override
    public void simulationPeriodic() {
        // REV reports the duty cycle the Spark applied; the Vortex then spins up like any other motor
        double batteryVolts = RobotController.getBatteryVoltage();
        double dt = TimedRobot.kDefaultPeriod;
        simFeedVolts = feedVortexSim.getAppliedOutput() * batteryVolts;
        simWheelVolts = wheelVortexSim.getAppliedOutput() * batteryVolts;

        feedMotorSim.setInputVoltage(simFeedVolts);
        feedMotorSim.update(dt);
        feedVortexSim.iterate(feedMotorSim.getAngularVelocityRPM(), batteryVolts, dt);

        wheelMotorSim.setInputVoltage(simWheelVolts);
        wheelMotorSim.update(dt);
        wheelVortexSim.iterate(wheelMotorSim.getAngularVelocityRPM(), batteryVolts, dt);
    }

    // for the Fieldrunner game bridge: the voltage the spindex feed is running at, negative feeds the launcher
    public double frGetSimFeedVolts() {
        return simFeedVolts;
    }

    // for the Fieldrunner game bridge: the voltage the spindex wheel is running at, negative turns the hopper forward
    public double frGetSimWheelVolts() {
        return simWheelVolts;
    }

    // makes a Vortex with its load as a DC motor sim
    private static DCMotorSim makeSimVortex() {
        DCMotor vortex = DCMotor.getNeoVortex(1);
        return new DCMotorSim(
                LinearSystemId.createDCMotorSystem(vortex, 0.0005, 1), vortex); // 0.0005 kg m^2: the Vortex with its load
    }

    public static class Tester extends PartialRobot {
        private final SpinDexSubsystem spindex = new SpinDexSubsystem();

        public Tester() {
            controller.a().whileTrue(spindex.getRun());

        }
    }
}
