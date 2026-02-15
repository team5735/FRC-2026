package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
  public final TalonFX talon = new TalonFX(Constants.mot1);


  public ClimberSubsystem() {
    talon.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
  }
  private final DigitalInput limitUp = new DigitalInput(Constants.upperLimitPin);
  private final DigitalInput limitDown = new DigitalInput(Constants.lowerLimitPin);

  public void climbUp() {
    if (!limitUp.get()){
      talon.setVoltage(Constants.upVolts);
    }else {talon.setVoltage(0);}
    
  }

  public void climbDown(){
    if (!limitDown.get()){
      talon.setVoltage(Constants.downVolts);
    }else {talon.setVoltage(0);}
  
  }

  public void stop(){
    talon.setVoltage(0);
  }

  @Override
  public void periodic(){
    

    //for elastic
    SmartDashboard.putBoolean("limitDown", limitDown.get());
    SmartDashboard.putBoolean("limitUp", limitUp.get());

    double current = talon.getSupplyCurrent().getValueAsDouble();
    SmartDashboard.putNumber("Climber/Current", current);

    double velocity = talon.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("Climber/Velocity", velocity);

    double forwardLimit = talon.getForwardLimit().getValueAsDouble();
    SmartDashboard.putNumber("Climber/ForwardLimit", forwardLimit);

    double reverseLimit = talon.getReverseLimit().getValueAsDouble();
    SmartDashboard.putNumber("Climber/ReverseLimit", reverseLimit);


  }
}
    

