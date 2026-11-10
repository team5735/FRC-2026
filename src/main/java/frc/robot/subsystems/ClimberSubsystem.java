package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PartialRobot;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;

public class ClimberSubsystem extends SubsystemBase {
    public final TalonFX talon = new TalonFX(Constants.CLIMB_TALON_ID);

    private final DigitalInput limitDown = new DigitalInput(Constants.CLIMB_UPPER_LIMIT_PIN);
    private final DigitalInput limitUp = new DigitalInput(Constants.CLIMB_LOWER_LIMIT_PIN);
    private boolean canMoveUp;
    private boolean canMoveDown;
    private boolean overriden;

    // simulation: rotor rotations of extension; the winch is clamped between its two end switches
    private final TalonFXSimState talonSim = talon.getSimState();
    private static final double SIM_STROKE_ROTATIONS = 40; // rotor rotations from fully retracted to fully extended
    private final DCMotorSim winchSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    0.3125 / (2 * Math.PI), // kV, V per (rotor rotation/s); 5 V runs a full stroke in about 2.7 s
                    0.0125 / (2 * Math.PI)), // kA, V per (rotor rotation/s^2)
            DCMotor.getKrakenX60(1));
    private final DIOSim limitDownSim = new DIOSim(limitDown); // reads false at the bottom
    private final DIOSim limitUpSim = new DIOSim(limitUp); // reads false at the top

    public ClimberSubsystem() {
        super();
        talon.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    public Command getExtendCommand() {
        return run(() -> extend()).finallyDo(a -> stop());
    }

    public Command getRetractCommand() {
        return run(() -> retract()).finallyDo(a -> stop());
    }

    public Command getExtendOverrideCommand() {
        return run(() -> extendOverride()).finallyDo(a -> stop());
    }

    public Command getRetractOverrideCommand() {
        return run(() -> retractOverride()).finallyDo(a -> stop());
    }

    public void retract() {
        if (canMoveUp) {
            talon.setVoltage(ClimberConstants.RETRACT_VOLTS);
        } else {
            talon.setVoltage(0);
        }
    }

    public void extend() {
        if (canMoveDown) {
            talon.setVoltage(ClimberConstants.EXTEND_VOLTS);
        } else {
            talon.setVoltage(0);
        }
    }

    public Command getFullyExtendCommand() {
        return run(() -> extend()).until(this::isAtUpLimit).finallyDo(() -> stop());
    }

    public Command getFullyDetractCommand() {
        return run(() -> retract()).until(this::isAtDownLimit).finallyDo(() -> stop());
    }

    public void retractOverride() {
        overriden = true;
        talon.setVoltage(ClimberConstants.RETRACT_VOLTS);
    }

    public void extendOverride() {
        overriden = true;
        talon.setVoltage(ClimberConstants.EXTEND_VOLTS);
    }

    public void stop() {
        talon.setVoltage(0);
        overriden = false;
    }

    public boolean isAtDownLimit() {
        return !limitDown.get();
    }

    public boolean isAtUpLimit() {
        return !limitUp.get();
    }

    @Override
    public void periodic() {
        // DIO
        canMoveUp = limitDown.get();
        canMoveDown = limitUp.get();

        if (!canMoveDown && talon.getMotorVoltage().getValueAsDouble() < 0 && !overriden) {
            talon.setVoltage(0);
        } else if (!canMoveUp && talon.getMotorVoltage().getValueAsDouble() > 0 && !overriden) {
            talon.setVoltage(0);
        }

        SmartDashboard.putBoolean("limitUp", limitUp.get());
        SmartDashboard.putBoolean("limitDown", limitDown.get());

        double current = talon.getSupplyCurrent().getValueAsDouble();
        SmartDashboard.putNumber("Climber/Current", current);

        double velocity = talon.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Climber/Velocity", velocity);

        double forwardLimit = talon.getForwardLimit().getValueAsDouble();
        SmartDashboard.putNumber("Climber/ForwardLimit", forwardLimit);

        double reverseLimit = talon.getReverseLimit().getValueAsDouble();
        SmartDashboard.putNumber("Climber/ReverseLimit", reverseLimit);
    }

    // ---- simulation

    @Override
    public void simulationPeriodic() {
        // negative motor voltage extends, so the winch is driven by the flipped voltage; static friction is a dead
        // band: the winch needs kS volts before it moves, then the rest drives it
        talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double volts = -talonSim.getMotorVoltage();
        double kS = 0.3; // volts
        double drivingVolts = 0;
        if (Math.abs(volts) > kS) {
            drivingVolts = volts - Math.copySign(kS, volts);
        }
        winchSim.setInputVoltage(drivingVolts);
        winchSim.update(TimedRobot.kDefaultPeriod);

        // hard stops at the ends of the stroke
        double rotations = winchSim.getAngularPositionRotations();
        if (rotations < 0) {
            winchSim.setState(0, 0);
        }
        if (rotations > SIM_STROKE_ROTATIONS) {
            winchSim.setState(Units.rotationsToRadians(SIM_STROKE_ROTATIONS), 0);
        }

        // the rotor turns backwards while the climber extends
        double speedRps = Units.radiansToRotations(winchSim.getAngularVelocityRadPerSec());
        talonSim.setRawRotorPosition(-winchSim.getAngularPositionRotations());
        talonSim.setRotorVelocity(-speedRps);

        // end switches read false when engaged
        double travel = frGetSimExtension();
        double switchBand = 0.002; // fraction of the stroke within which an end switch reads on
        boolean isRetracted = travel <= switchBand;
        boolean isExtended = travel >= 1 - switchBand;
        limitDownSim.setValue(!isRetracted);
        limitUpSim.setValue(!isExtended);
    }

    // for the Fieldrunner game bridge: the climber's true travel, 0 fully retracted to 1 fully extended
    public double frGetSimExtension() {
        return winchSim.getAngularPositionRotations() / SIM_STROKE_ROTATIONS;
    }

    public static class Tester extends PartialRobot {

        ClimberSubsystem climber = new ClimberSubsystem();

        public Tester() {
            super();
            controller.rightTrigger().whileTrue(climber.getExtendCommand());
            controller.leftTrigger().whileTrue(climber.getRetractCommand());
        }
    };
}
