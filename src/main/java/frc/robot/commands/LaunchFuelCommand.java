package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelLauncherSubsystem;

// Start and stops the motor
public class LaunchFuelCommand extends Command {

    private FuelLauncherSubsystem launcher;

    public LaunchFuelCommand(FuelLauncherSubsystem launcher) {
        this.launcher = launcher;
    }

    @Override
    public void initialize() {
        launcher.activateVoltage();
    }

    @Override
    public void end(boolean interrupted) {
        launcher.deactivateVoltage();
    }

}