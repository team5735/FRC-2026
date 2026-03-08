// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.PartialRobot;
import frc.robot.constants.Constants;
import frc.robot.constants.FuelLauncherConstants;
import frc.robot.util.NTable;
import frc.robot.util.TunablePIDController;

public class FuelLauncherSubsystem extends SubsystemBase {
    private final TalonFX krakenLeft = new TalonFX(Constants.LAUNCHER_LEFT_KRAKEN_ID);
    private final TalonFX krakenRight = new TalonFX(Constants.LAUNCHER_RIGHT_KRAKEN_ID);

    private final TunablePIDController pid = new TunablePIDController("fuel_launcher");
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(FuelLauncherConstants.KS,
            FuelLauncherConstants.KV);

    private final NTable table = NTable.root("fuel_launcher");

    public FuelLauncherSubsystem() {
        super();
        SmartDashboard.putNumber("shooter_volts", FuelLauncherConstants.LAUNCHER_VOLTS);
        krakenLeft.getConfigurator()
                .apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));
        krakenRight.setControl(new Follower(Constants.LAUNCHER_LEFT_KRAKEN_ID, MotorAlignmentValue.Opposed));

        pid.ensureP(FuelLauncherConstants.KP);
        pid.ensureTolerance(FuelLauncherConstants.RPM_TOLERANCE);
        pid.setup(0);
    }

    public double getRPM() {
        SmartDashboard.putNumber("launcher/motorRPS", krakenLeft.getVelocity().getValueAsDouble());
        return krakenLeft.getVelocity().getValue().in(RPM) * FuelLauncherConstants.GEARING;
    }

    private void setTargetRPM(double rpm) {
        pid.setup(rpm);
    }

    public void usePID() {
        double output = pid.calculate(getRPM()) + ff.calculate(pid.getController().getSetpoint());
        SmartDashboard.putNumber("launcher/output", output);
        krakenLeft.setVoltage(output);
    }

    LinearFilter filter = LinearFilter.movingAverage(50);

    double lastMax = 0, lastMin = 0;
    double realSetpoint = 0;
    int ticksSinceMax = 0, ticksSinceMin = 0;
    LinearFilter errorAverage = LinearFilter.movingAverage(50);

    @Override
    // Multiplying krakenMotor by 60 to turn rotations per second into rotations per
    // minute (RPM)
    public void periodic() {
        double currentRPM = getRPM();
        this.table.set("speed_rpm", currentRPM);
        this.table.set("target_speed", pid.getController().getSetpoint());
        this.table.set("moving_average", filter.calculate(currentRPM));

        ticksSinceMax++;
        ticksSinceMin++;
        double error = Math.abs(currentRPM - realSetpoint);
        if (error > lastMax || ticksSinceMax >= 50) {
            lastMax = error;
            ticksSinceMax = 0;
        }
        if (error < lastMin || ticksSinceMin >= 50) {
            lastMin = error;
            ticksSinceMin = 0;
        }
        NTable errorTable = this.table.sub("error");
        errorTable.set("mean", errorAverage.calculate(error));
        errorTable.set("max", lastMax);
        errorTable.set("min", lastMin);
    }

    public Command getLaunchFuel(AngularVelocity speed) {
        return runOnce(this::retunePID)
                .andThen(
                        startRun(() -> {
                            this.realSetpoint = speed.in(RPM);
                            setTargetRPM(speed.in(RPM));
                        }, this::usePID))
                .withName("Launch Fuel at " + speed);
    }

    public Command getFedLaunch(SpinDexSubsystem spindex, AngularVelocity speed) {
        return this.getLaunchFuel(speed).until(pid::atSetpoint)
                .andThen(this.getLaunchFuel(speed).alongWith(spindex.getRun()));
    }

    private SysIdRoutine routine = new SysIdRoutine(
            new Config(Volts.of(0.5).per(Second), Volts.of(4), null, null),
            new Mechanism(v -> krakenLeft.setVoltage(v.in(Volts)), log -> {
                log.motor("launcher").voltage(krakenLeft.getMotorVoltage().getValue())
                        .angularVelocity(RPM.of(getRPM())).angularPosition(krakenLeft.getPosition().getValue());
            }, this));

    public Command sysIdDynamic(Direction direction) {
        return routine.dynamic(direction);
    }

    public Command sysIdQuasistatic(Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command runAtSpeed(AngularVelocity speed) {
        return startRun(() -> pid.setup(speed.in(RPM)), this::usePID);
    }

    public void retunePID() {
        pid.setup(pid.getController().getSetpoint());
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    public static class Tester extends PartialRobot {
        private final SpinDexSubsystem spindex = new SpinDexSubsystem();
        private final FuelLauncherSubsystem launcher = new FuelLauncherSubsystem();
        private final IntakeSubsystem intake = new IntakeSubsystem();

        public Tester() {
            super();
            launcher.setDefaultCommand(launcher.getLaunchFuel(RPM.of(0)));

            controller.a().whileTrue(launcher.getFedLaunch(spindex, RPM.of(3000)));
            controller.x().whileTrue(spindex.startEnd(spindex::reverseWheel, spindex::stopWheel));
            controller.rightBumper().whileTrue(intake.getIntakeForwardRollCommand());
        }
    }
}
