package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelLauncherSubsystem;

public class PIDIntakeCommand extends Command {
    private double targetVelocity;
    private FuelLauncherSubsystem launcher;


    public PIDIntakeCommand(double targetVelocity, FuelLauncherSubsystem launcher) {
        this.launcher = launcher;
        this.targetVelocity = targetVelocity;
    }

    @Override
    public void initialize() {
        launcher.setGoal(targetVelocity);
    }
    
    @Override
    public void execute() {
        launcher.usePID();
    }

    // @Override
    // public void end(boolean interrupted) {
    //     launcher.intake_stop();
    // }

    @Override
    public boolean isFinished() {
      return false;
    }    
}
