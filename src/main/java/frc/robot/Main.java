// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.HoodSubsystem;

// Here be dragons.
public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        TimedRobot bot;

        switch (Constants.CURRENT_ROBOT){
            case FULL_ROBOT:
                bot = new Robot();
                break;
            case HOOD:
                bot = new HoodSubsystem.HoodTuningBot();
                break;
            case HOOD_PEEK_A_BOO:
                bot = new HoodSubsystem.HoodPeekABooBot();
                break;
            default:
                throw new IllegalStateException("Unknown robot configuration: " 
                                                + Constants.CURRENT_ROBOT);
        }
        RobotBase.startRobot(()->bot);
    }
}
