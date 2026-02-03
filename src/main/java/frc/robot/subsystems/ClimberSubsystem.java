package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
  public final TalonFX talon = new TalonFX(Constants.mot1);
  public boolean fullExtension = false;

  public void climbUp() {

    talon.setVoltage(Constants.upVolts);
  }

  public void climbDown(){
    talon.setVoltage(Constants.downVolts);
  }

  public void stop(){
    talon.setVoltage(0);
  }

  @Override
    public void periodic() {
        
      if (fullExtension) stop();

    }

}
    

