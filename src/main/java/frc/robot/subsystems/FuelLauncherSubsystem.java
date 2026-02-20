// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.Constants;
import frc.robot.constants.FuelLauncherConstants;
import frc.robot.util.TunablePIDController;

public class FuelLauncherSubsystem extends SubsystemBase {
    private final TalonFX krakenLeft = new TalonFX(Constants.LAUNCHER_LEFT_KRAKEN_ID);
    private final TalonFX krakenRight = new TalonFX(Constants.LAUNCHER_RIGHT_KRAKEN_ID);

    private final TunablePIDController pid = new TunablePIDController("fuel_launcher", 0, 0, 0);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(FuelLauncherConstants.KS,
            FuelLauncherConstants.KV);

    public FuelLauncherSubsystem() {
        SmartDashboard.putNumber("shooter_volts", FuelLauncherConstants.LAUNCHER_VOLTS);
        krakenLeft.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast));
        krakenRight.setControl(new Follower(Constants.LAUNCHER_LEFT_KRAKEN_ID, MotorAlignmentValue.Opposed));
        pid.setup(0);
    }

    public double getRPM() {
        SmartDashboard.putNumber("launcher/motorRPS", krakenLeft.getVelocity().getValueAsDouble());
        return krakenLeft.getVelocity().getValue().in(RPM) * FuelLauncherConstants.GEARING;
    }

    private void activateVoltage() {
        krakenLeft.setVoltage(SmartDashboard.getNumber("shooter_volts", FuelLauncherConstants.LAUNCHER_VOLTS));
    }

    private void deactivateVoltage() {
        krakenLeft.setVoltage(0);
    }

    private void setTargetRPM(double rpm) {
        pid.setup(rpm, FuelLauncherConstants.RPM_TOLERANCE);
    }

    private void usePID() {
        double output = pid.calculate(getRPM()) + ff.calculate(pid.getController().getSetpoint());
        SmartDashboard.putNumber("launcher/output", output);
        krakenLeft.setVoltage(output);
    }

    LinearFilter filter = LinearFilter.movingAverage(50);

    @Override
    public void periodic() {
        double currentRPM = getRPM(); 
        SmartDashboard.putNumber("launcher/speed_rpm", currentRPM);
        SmartDashboard.putNumber("launcher/target_speed", pid.getController().getSetpoint());
        SmartDashboard.putNumber("launcher/moving_average", filter.calculate(currentRPM));
    }

    public Command launchFuel(AngularVelocity speed) {
        return startRun(() -> setTargetRPM(speed.in(RPM)), this::usePID);
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

    public Command sysIdQuasistatic(Direction direction){
        return routine.quasistatic(direction);
    }

    public Command runAtSpeed(AngularVelocity speed){
        return startRun(() -> pid.setup(speed.in(RPM)), this::usePID);
    }

    public Command retunePID() {
        return Commands.runOnce(() -> pid.setup(pid.getController().getSetpoint()));
    }
}