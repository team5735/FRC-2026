package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
  public final TalonFX talon = new TalonFX(Constants.mot1);

  public ClimberSubsystem() {
    talon.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
  }
  private final DigitalInput limitUp = new DigitalInput(Constants.upperLimitPin);

  public void climbUp() {
    talon.setVoltage(Constants.upVolts);
  }

  public void climbDown(){
    talon.setVoltage(Constants.downVolts);
  }

  public void stop(){
    talon.setVoltage(0);
  }
}
    

