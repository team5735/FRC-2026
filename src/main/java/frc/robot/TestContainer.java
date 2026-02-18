package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.MoveCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.HoodSubsystem;

public class TestContainer {
    private static final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    private static final HoodSubsystem hood = new HoodSubsystem();

    public static void configureBindings() {
        driveController.a().onTrue(new MoveCommand(hood, 0.6));
        driveController.a().onFalse(new MoveCommand(hood, 0.4));
    }
}
