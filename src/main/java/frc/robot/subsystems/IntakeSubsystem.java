// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.PartialRobot;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeSlapdown = new TalonFX(Constants.INTAKE_SLAPDOWN_TALONFX_ID);
    private final TalonFX intakeRoller = new TalonFX(Constants.INTAKE_ROLLER_TALONFX_ID);
    private final ArmFeedforward ff = new ArmFeedforward(IntakeConstants.KS, IntakeConstants.KG, IntakeConstants.KV);
    private final DigitalInput hallLimit = new DigitalInput(Constants.INTAKE_LIMIT_PIN);

    // simulation: the arm rests stowed against its upper stop, which is where the hall switch turns on (LIMIT_POS)
    private static final double SIM_STOWED_RADIANS = Units.degreesToRadians(IntakeConstants.LIMIT_POS.in(Degrees));

    // arm: radians above horizontal; the plant is in mechanism units, the hard stops are built in
    private final TalonFXSimState slapdownSim = intakeSlapdown.getSimState();
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                    IntakeConstants.KV / (2 * Math.PI),
                    0.05 / (2 * Math.PI)), // kA, V per (rotation/s^2), not characterized
            DCMotor.getKrakenX60(1), IntakeConstants.GEAR_REDUCTION,
            0.6, // arm length, meters; sets gravity so holding horizontal costs about kG
            0, // lower hard stop at horizontal, radians
            SIM_STOWED_RADIANS, true, SIM_STOWED_RADIANS);
    private final DIOSim hallLimitSim = new DIOSim(hallLimit);

    // roller: roller rotations
    private final TalonFXSimState rollerSim = intakeRoller.getSimState();
    private final DCMotorSim rollerMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    0.12 / (2 * Math.PI), // kV, V per (rotation/s)
                    0.005 / (2 * Math.PI)), // kA, V per (rotation/s^2)
            DCMotor.getKrakenX60(1));
    private double simRollerVolts = 0; // last voltage the roller Talon applied, mechanism frame

    public IntakeSubsystem() {
        super();
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 10;
        intakeRoller.getConfigurator().apply(rollerConfig);

        TalonFXConfiguration slapdownConfig = new TalonFXConfiguration();
        slapdownConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        slapdownConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // TODO swap back to brake
        slapdownConfig.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_REDUCTION;
        intakeSlapdown.getConfigurator().apply(slapdownConfig);

        intakeSlapdown.setPosition(IntakeConstants.START_POS);
    }

    public void forwardRoll() {
        intakeRoller.setVoltage(IntakeConstants.ROLL_IN_VOLTS);
    }

    public void reverseRoll() {
        intakeRoller.setVoltage(IntakeConstants.ROLL_OUT_VOLTS);
    }

    public void stopRoll() {
        intakeRoller.setVoltage(0);
    }

    private void runSlapdown(AngularVelocity velocity) {
        intakeSlapdown.setVoltage(ff.calculate(getSlapdownPosition().in(Radians), velocity.in(RotationsPerSecond)));
    }

    public boolean isAtPosition(Angle position) {
        return getSlapdownPosition().isNear(position, IntakeConstants.ANGULAR_TOLERANCE);
    }

    public Angle getSlapdownPosition() {
        return intakeSlapdown.getPosition().getValue();
    }

    public Command getIntakeForwardRollCommand() {
        return startEnd(() -> forwardRoll(), () -> stopRoll());
    }

    public Command getIntakeReverseRollCommand() {
        return startEnd(() -> reverseRoll(), () -> stopRoll());
    }

    public Command getLiftCommand() {
        return runEnd(() -> runSlapdown(IntakeConstants.LIFT_VEL), () -> intakeSlapdown.setVoltage(0))
                .until(() -> getSlapdownPosition().gte(IntakeConstants.UPPER_RELEASE_POS));
    }

    public Command getSlapdownCommand() {
        return runEnd(() -> runSlapdown(IntakeConstants.SLAPDOWN_VEL), () -> intakeSlapdown.setVoltage(0))
                .until(() -> getSlapdownPosition().lte(IntakeConstants.LOWER_RELEASE_POS));
    }
    public Command getStopRollCommand() {
        return Commands.runOnce(() -> stopRoll());
    }
    public Command zeroSlapdownPosition() {
        return Commands.runOnce(() -> intakeSlapdown.setPosition(IntakeConstants.LIMIT_POS)).ignoringDisable(true);
    }

    public SysIdRoutine routine = new SysIdRoutine(
            new Config(Volts.of(0.1).per(Second), Volts.of(0.5), null),
            new Mechanism(v -> intakeSlapdown.setVoltage(v.in(Volts)), log -> {
                log.motor("slapdown").angularPosition(getSlapdownPosition())
                        .angularVelocity(intakeSlapdown.getVelocity().getValue()).voltage(intakeSlapdown.getMotorVoltage().getValue());
            }, this));

    public Command sysIdQuasistatic(Direction direction){
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction){
        return routine.dynamic(direction);
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intake/limit_engaged", !hallLimit.get());
        SmartDashboard.putNumber("intake/slapdown_volts", intakeSlapdown.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("intake/slapdown_pos_deg", getSlapdownPosition().in(Degrees));
        SmartDashboard.putNumber("intake/slapdown_vel_dps", intakeSlapdown.getVelocity().getValue().in(DegreesPerSecond));
    }

    public Trigger limitEngaged = new Trigger(() -> !hallLimit.get());

    // ---- simulation

    @Override
    public void simulationPeriodic() {
        double batteryVolts = RobotController.getBatteryVoltage();
        double dt = TimedRobot.kDefaultPeriod;

        // arm: static friction as a dead band (the arm needs kS volts before it moves, then the rest drives it)
        slapdownSim.setSupplyVoltage(batteryVolts);
        double armVolts = slapdownSim.getMotorVoltage();
        double armDrivingVolts = 0;
        if (Math.abs(armVolts) > IntakeConstants.KS) {
            armDrivingVolts = armVolts - Math.copySign(IntakeConstants.KS, armVolts);
        }
        armSim.setInputVoltage(armDrivingVolts);
        armSim.update(dt);

        // the arm's rotor counts from the stowed hard stop
        double armMovedRotations = Units.radiansToRotations(armSim.getAngleRads() - SIM_STOWED_RADIANS);
        double armSpeedRps = Units.radiansToRotations(armSim.getVelocityRadPerSec());
        slapdownSim.setRawRotorPosition(IntakeConstants.GEAR_REDUCTION * armMovedRotations);
        slapdownSim.setRotorVelocity(IntakeConstants.GEAR_REDUCTION * armSpeedRps);

        // roller: the Talon is Clockwise_Positive, so the voltage that spins the roller is the Talon's, flipped
        rollerSim.setSupplyVoltage(batteryVolts);
        double rollerVolts = -rollerSim.getMotorVoltage();
        simRollerVolts = rollerVolts;
        double rollerKS = 0.1; // volts
        double rollerDrivingVolts = 0;
        if (Math.abs(rollerVolts) > rollerKS) {
            rollerDrivingVolts = rollerVolts - Math.copySign(rollerKS, rollerVolts);
        }
        rollerMotorSim.setInputVoltage(rollerDrivingVolts);
        rollerMotorSim.update(dt);

        double rollerSpeedRps = Units.radiansToRotations(rollerMotorSim.getAngularVelocityRadPerSec());
        rollerSim.setRawRotorPosition(-rollerMotorSim.getAngularPositionRotations());
        rollerSim.setRotorVelocity(-rollerSpeedRps);

        // the hall switch reads false when engaged
        double hallMarginRadians = Units.degreesToRadians(0.1); // reads on within 0.1 deg of the stowed stop
        boolean isHallOn = armSim.getAngleRads() >= SIM_STOWED_RADIANS - hallMarginRadians;
        hallLimitSim.setValue(!isHallOn);
    }

    // for the Fieldrunner game bridge: the intake arm's true angle above horizontal
    public Angle frGetSimArmAngle() {
        return Radians.of(armSim.getAngleRads());
    }

    // for the Fieldrunner game bridge: the voltage the intake roller is running at, positive pulls balls in
    public double frGetSimRollerVolts() {
        return simRollerVolts;
    }

    public static class Tester extends PartialRobot {
        private final IntakeSubsystem intake = new IntakeSubsystem();

        public Tester() {
            super();
            intake.limitEngaged.onTrue(intake.zeroSlapdownPosition());
            
            controller.rightBumper().whileTrue(intake.getLiftCommand());
            controller.leftBumper().whileTrue(intake.getSlapdownCommand());

            controller.a().whileTrue(intake.sysIdDynamic(Direction.kForward)); // forward = up
            controller.b().whileTrue(intake.sysIdDynamic(Direction.kReverse));
            controller.x().whileTrue(intake.sysIdQuasistatic(Direction.kForward));
            controller.y().whileTrue(intake.sysIdQuasistatic(Direction.kReverse));
        }
    }
}

    
