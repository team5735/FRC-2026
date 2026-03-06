// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.Constants;
import frc.robot.constants.robot.CompbotTunerConstants;
import frc.robot.constants.robot.DevbotTunerConstants;
import frc.robot.subsystems.HoodSubsystem;

// Here be dragons.
public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(() -> switch (Constants.CURRENT_ROBOT) {
            case FULL_DEVBOT -> new Robot(DevbotTunerConstants.createDrivetrain());
            case FULL_COMPBOT -> new Robot(CompbotTunerConstants.createDrivetrain());
            case HOOD -> HoodSubsystem.tester;
            case HOOD_PEEK_A_BOO -> HoodSubsystem.peekABooBot;
        });
    }
}
