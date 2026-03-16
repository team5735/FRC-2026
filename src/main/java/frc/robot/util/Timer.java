package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.Constants;

public class Timer {
    private long startTime;
    private NTable table;     // optional write to this NTable
    private String fieldName; // location upon calls to toc

    public Timer(){
        startTime = RobotController.getFPGATime();
    }

    public Timer(String name){
        table = NTable.root().sub("Timer");
        fieldName = name;
        startTime = RobotController.getFPGATime();
    }
    public Timer(NTable tab, String name){
        table = tab;
        fieldName = name;
        startTime = RobotController.getFPGATime();
    }

    public double ntoc(){
        double v = msToc();
        table.setSimple(fieldName, v);
        return v;
    }

    public double toc(){
        if (Constants.enableNTableTimerBasedProfiling && table != null)
            return ntoc();
        else
            return msToc();
    }

    public long usToc(){
        return RobotController.getFPGATime() - startTime;
    }

    public double msToc(){
        return usToc()/1000.0;
    }
}
