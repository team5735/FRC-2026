// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.Constants;

// Here be dragons.
public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        if (Constants.CURRENT_ROBOT == Constants.Running.FULL_ROBOT) {
            RobotBase.startRobot(Robot::new);
        } else {
            RobotBase.startRobot(Constants.RUNNABLE_SUBSYSTEMS.get(Constants.CURRENT_ROBOT));
        }
    }
}
