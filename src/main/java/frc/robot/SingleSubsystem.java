package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.util.NTable;

public class SingleSubsystem extends TimedRobot {
    public final CommandXboxController driveController = new CommandXboxController(Constants.DRIVE_CONTROLLER_PORT);

    public static final ClimberSubsystem climber = new ClimberSubsystem();

    SingleSubsystem() {
        driveController.a().whileTrue(climber.getClimbUpCommand());
        driveController.b().whileTrue(climber.getClimbDownCommand());
        driveController.x().onTrue(climber.getFullyExtendCommand());
        driveController.y().onTrue(climber.getFullyDetractCommand());
        driveController.leftBumper().whileTrue(climber.getOverrideCommand());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        NTable.updateAllSendables();
    }
}
