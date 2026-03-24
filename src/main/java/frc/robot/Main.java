// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.Constants;
import frc.robot.constants.robot.CompbotTunerConstants;
import frc.robot.constants.robot.DevbotTunerConstants;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SpinDexSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// Here be dragons.
public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        SignalLogger.enableAutoLogging(false);
        StatusLogger.disableAutoLogging();
        RobotBase.startRobot(() -> switch (Constants.CURRENT_ROBOT) {
            case FULL_DEVBOT -> new Robot(DevbotTunerConstants.createDrivetrain());
            case FULL_COMPBOT -> new Robot(CompbotTunerConstants.createDrivetrain());
            case HOOD -> new HoodSubsystem.Tester();
            case INTAKE -> new IntakeSubsystem.Tester();
            case CLIMBER -> new ClimberSubsystem.Tester();
            case TURRET -> new TurretSubsystem.Tester();
            case TURRET_AIMING -> new TurretSubsystem.AimingTest();
            case SPINDEX -> new SpinDexSubsystem.Tester();
            case LAUNCHER -> new LauncherSubsystem.Tester();
            case CANDLE -> new CANdleSubsystem.Tester();
            case INTERPOLATION -> new InterpolationRobot();
        });
    }
}
