package frc.robot.constants;

public class LauncherConstants {
    public static final double RPM_TOLERANCE = 35; // 35 is 1% of expected RPM

    public static final double KP = 0.003;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double KS = 0.2;
    public static final double KV = 0.00084853;

    public static final double GEARING = 36./15; //input 36 tooth pulley, output 15

    public static final int LAUNCHER_VOLTS = 4;

    public static final double DEFAULT_SETPOINT = 3000 - 24;
}
