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
}