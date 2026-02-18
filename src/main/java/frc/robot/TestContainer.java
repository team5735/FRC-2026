package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SpinDexSubsystem;

public class TestContainer {
    private static final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    private static final SpinDexSubsystem spindex = new SpinDexSubsystem();

    public static void configureBindings() {
        driveController.b().onTrue(spindex.getStart());
    }
}
