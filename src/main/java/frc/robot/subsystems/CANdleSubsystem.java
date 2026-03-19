package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.PartialRobot;

import frc.robot.constants.Constants;

public class CANdleSubsystem extends SubsystemBase {

    private final CANdle candle = new CANdle(Constants.CANDLE_ID);

    private final SolidColor colorRequest = new SolidColor(0, 399);

    public CANdleSubsystem() {
        candle.getConfigurator().apply(
            new CANdleConfiguration().withLED(
                new LEDConfigs().withStripType(StripTypeValue.GRB)
            )
        );
    }

    public Command setColor(Color color) {
        return runOnce(() -> 
            candle.setControl(
                colorRequest.withColor(new RGBWColor(color))
            )
        );
    }

        // This is a full robot config for testing the hood subsystem
    public static class Tester extends PartialRobot {
        
    public CANdleSubsystem CANdle = new CANdleSubsystem();

        public Tester() {
            super();

        controller.x().onTrue(CANdle.setColor(new Color(0.0, 1.0, 0.0))); // This is the custom color for orange
        controller.a().onTrue(CANdle.setColor(Color.kRed));              
        controller.b().onTrue(CANdle.setColor(Color.kBlue)); 
        controller.y().onTrue(CANdle.setColor(new Color(0.0, 0.0, 0.0))); // This is the custom color for off

        }

        @Override
        public void robotPeriodic() {
            super.robotPeriodic();
        }

    };
}