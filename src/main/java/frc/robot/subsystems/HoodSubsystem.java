package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    private final Servo servo = new Servo(0);  
    private final AnalogInput feedback = new AnalogInput(0);

    // Runs one time when the robot starts
    public HoodSubsystem() {
        setPosition(0.4);  // default safe position at startup
}

    //Pos is set anywhere from 0 to 1
    public void setPosition(double position) {
    double safePosition = Math.max(0.1, Math.min(0.9, position));
    servo.set(safePosition);
}

    // Returns raw voltage from analog feedback wire
    public double getVoltage() {
        return feedback.getVoltage();
    }


    // Converts voltage (0-5V) into 0.0–1.0 normalized position
    public double getNormalizedPosition() {
        return feedback.getVoltage() / 5.0;
    }

    // Runs every 20ms automatically
    @Override
    public void periodic() {
        
    }
}