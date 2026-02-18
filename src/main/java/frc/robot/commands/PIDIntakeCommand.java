package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelLauncherSubsystem;

public class PIDIntakeCommand extends Command {
    private double targetRPM;
    private FuelLauncherSubsystem launcher;


    public PIDIntakeCommand(double targetRPM, FuelLauncherSubsystem launcher) {
        this.launcher = launcher;
        this.targetRPM = targetRPM;
    }

    @Override
    public void initialize() {
        launcher.setTargetRPM(this.targetRPM);
    }
    
    @Override
    public void execute() {
        launcher.usePID();
    }

    @Override
    public boolean isFinished() {
      return false;
    }    
}
