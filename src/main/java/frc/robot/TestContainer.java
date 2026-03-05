package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LaunchFuelCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.FuelLauncherSubsystem;
import frc.robot.subsystems.SpinDexSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class TestContainer {
    private static final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    private static final FuelLauncherSubsystem launcher = new FuelLauncherSubsystem();
    private static final SpinDexSubsystem spindex = new SpinDexSubsystem();

    public static void configureBindings() {
        driveController.b().whileTrue(spindex.getStart());
        driveController.b().whileTrue(new LaunchFuelCommand(launcher));

        new Trigger(() -> launcher.getMotorRPM() >= 1000)
                .whileTrue(spindex.getStart());

    }
}
