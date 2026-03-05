package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;

public class TestContainer {
    private static final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    // private static final HoodSubsystem hood = new HoodSubsystem();

    public static void configureBindings() {
        // driveController.a().onTrue(hood.runOnce(() -> hood
        // .setPosition(SmartDashboard.getNumber("End Revolution Posiiton",
        // Constants.END_REVOLUTION_POSITION))));
        // driveController.a().onFalse(hood.runOnce(() -> hood.setPosition(
        // SmartDashboard.getNumber("Start Revolution Posiiton",
        // Constants.START_REVOLUTION_POSITION))));
    }
}
