package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinDexSubsystem;

public class SpinDexCommand extends Command {
    public final SpinDexSubsystem spindex;

    public SpinDexCommand(SpinDexSubsystem spindex) {
        this.spindex = spindex;
        addRequirements(spindex);
    }

    @Override
    public void initialize() {
        spindex.Motor6Run();
        spindex.Motor27Run();
    }

    @Override
    public void end(boolean interrupted) {
        spindex.Motor27stop();
        spindex.Motor6stop();
    }

}
