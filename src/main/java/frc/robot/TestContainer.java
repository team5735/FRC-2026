package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LaunchFuelCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelLauncherSubsystem;
import frc.robot.subsystems.SpinDexSubsystem;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.HoodSubsystem;

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
