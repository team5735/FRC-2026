package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.Constants;

/* Simple elapsed time keeper-tracker, used for profiling sections of code.
 * Priority is given to its simple usage. 
 * 
 * Used locally:
 * var T = new Timer();
 * double elapsed = T.toc();
 * 
 * Publish to network tables:
 * var T = new Timer("MyField");
 * ... slow code chunk ...
 * T.toc();
 * 
 * To disable all profiling telemetry publishing to network tables, set
 * Constants.enableNTableTimerBasedProfiling to false
 * This is useful during competitions to not hog resources with debug stuff
 */
public class Timer {
    private long startTime;
    private NTable table;     // optional write to this NTable
    private String fieldName; // location upon calls to toc

    /* create local only Timer */
    public Timer(){
        startTime = RobotController.getFPGATime();
    }

    /* create Timer with publishing to default network tables table 
     * and custom field
    */
    public Timer(String name){
        table = NTable.root().sub("Timer");
        fieldName = name;
        startTime = RobotController.getFPGATime();
    }

    /* create Timer with publishing to custom network tables table
     * and custom field
     */
    public Timer(NTable tab, String name){
        table = tab;
        fieldName = name;
        startTime = RobotController.getFPGATime();
    }

    /* tic/toc. This ends the timing. The initial "tic" is done at construction
     * This should be the only one you call. It will return the elapsed time in milliseconds
     * It will write to the network tables entry if one is provided, and the enable flag is set
     * returns the elapsed time in milliseconds
     */
    public double toc(){
        if (Constants.enableNTableTimerBasedProfiling && table != null)
            return ntoc();
        else
            return msToc();
    }

    /* version of toc that writes directly to network tables, no checks
     * returns the elapsed time in milliseconds
     * (you should probably not use this directly)
     */
    public double ntoc(){
        double v = msToc();
        table.setSimple(fieldName, v);
        return v;
    }


    /* version of toc that returns the elapsed time in microseconds
     * does not write to network tables
     * (you should probably not use this directly)
     */
    public long usToc(){
        return RobotController.getFPGATime() - startTime;
    }

    /* version of toc that returns the elapsed time in milliseconds
     * does not write to network tables
     * (you should probably not use this directly)
     */
    public double msToc(){
        return usToc()/1000.0;
    }
}
