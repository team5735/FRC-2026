// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.NTable;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in the TimedRobot documentation. If you change the name of
 * this class or the package after creating this project, you must also update
 * the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robot;

    public Robot() {
        NTable.root().set("breadboard mode", Constants.BREADBOARD_MODE);
        if (Constants.BREADBOARD_MODE) {
            TestContainer.configureBindings();
        } else {
            robot = new RobotContainer();
        }

        NTable.root().set("scheduler", CommandScheduler.getInstance());
    }

    @Override
    public void robotPeriodic() {
        if (!Constants.BREADBOARD_MODE) {
            for (LimelightSubsystem limelight : RobotContainer.limelights) {
                limelight.handleVisionMeasurement();
            }
        }

        CommandScheduler.getInstance().run();
        NTable.updateAllSendables();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        if (Constants.BREADBOARD_MODE) {
            return;
        }

        autonomousCommand = robot.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
