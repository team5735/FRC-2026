package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class TestContainer {
    private static final CommandXboxController driveController = new CommandXboxController(
            Constants.DRIVE_CONTROLLER_PORT);

    private static final ClimberSubsystem climber = new ClimberSubsystem();

    public static void configureBindings() {
        driveController.a().whileTrue(climber.getClimbUpCommand());
        driveController.b().whileTrue(climber.getClimbDownCommand());
        driveController.x().onTrue(climber.getFullyExtendCommand());
        driveController.y().onTrue(climber.getFullyDetractCommand());
        driveController.leftBumper().whileTrue(climber.getOverrideCommand());
    }
}
