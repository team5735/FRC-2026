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
        switch (Constants.CURRENT_ROBOT) {
            case FULL_ROBOT -> RobotBase.startRobot(Robot::new);
            case SINGLE_SUBSYSTEM -> RobotBase.startRobot(SingleSubsystem::new);
        }
    }
}
