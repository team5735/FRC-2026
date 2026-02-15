package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinDexSubsystem;
import frc.robot.util.TunablePIDController;

public class SpinDexCommand extends Command {
    public final SpinDexSubsystem spindex;

    public SpinDexCommand(SpinDexSubsystem spindex) {
        this.spindex = spindex;

    }

    @Override
    public void initialize() {
        spindex.talon6.setVoltage(4);
        spindex.talon27.setVoltage(2);

    }

    @Override
    public void execute() {

    }

}
