package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.FuelLauncherSubsystem;
import frc.robot.subsystems.SpinDexSubsystem;

// Start and stops the motor
public class LaunchFuelCommand extends Command {

    private FuelLauncherSubsystem launcher;

    private boolean SpindexInitiated = false;

    public SpinDexSubsystem spindex = new SpinDexSubsystem();

    public LaunchFuelCommand(FuelLauncherSubsystem launcher) {
        this.launcher = launcher;
    }

    // Starts
    @Override
    public void initialize() {
        launcher.activateVoltage();
    }

    @Override
    public void execute() {
        if (launcher.getMotorRPM() >= 1000 && !SpindexInitiated) {
            CommandScheduler.getInstance().schedule(new SpinDexCommand(spindex));
            SpindexInitiated = true;
        }
    }

    // Stops the motor
    @Override
    public void end(boolean interrupted) {
        launcher.deactivateVoltage();
    }

}