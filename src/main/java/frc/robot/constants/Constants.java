// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * 
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int LAUNCHER_VOLTS = 4;
    public static final int LAUNCHER_KRAKEN_ID = 1;
    public static final int LAUNCHER_LEFT_VORTEX_ID = 0;
    public static final int LAUNCHER_RIGHT_VORTEX_ID = 27;

    // For PIDs
    public static final double TOLERANCE = 0;
}
