package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
  public final TalonFX talon = new TalonFX(Constants.mot1);

  private final DigitalInput limitUp = new DigitalInput(Constants.upperLimitPin);
  private final DigitalInput limitDown = new DigitalInput(Constants.lowerLimitPin);
  private boolean canMoveUp;
  private boolean canMoveDown;


  public ClimberSubsystem() {
    talon.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
  }
  
  public Command getClimbDownCommand(){
    return run(()->climbDown()).finallyDo(a->stop());
  }

  public void climbUp() {
    if (canMoveUp){
      talon.setVoltage(Constants.upVolts);
    }else {talon.setVoltage(0);}
    
  }

  public void climbDown(){
    if (canMoveDown){
      talon.setVoltage(Constants.downVolts);
    }else {talon.setVoltage(0);}
  
  }

  public void stop(){
    talon.setVoltage(0);
  }

  public boolean isAtUpLimit(){
    return !limitUp.get();
  }

  public boolean isAtDownLimit(){
    return !limitDown.get();
  }

  @Override
  public void periodic(){
    //DIO
    canMoveUp=limitUp.get();
    canMoveDown=limitDown.get();

    if (!canMoveDown && talon.getMotorVoltage().getValueAsDouble()<0){
        talon.setVoltage(0);
    }else if (!canMoveUp && talon.getMotorVoltage().getValueAsDouble()>0){
        talon.setVoltage(0);
    }
    
    

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
    

