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
    private CANdle candle = new CANdle(Constants.CANDLE_ID);

    public CANdleSubsystem() {
        super();

        candle.getConfigurator().apply(
                new CANdleConfiguration().withLED(
                        new LEDConfigs().withStripType(StripTypeValue.GRB)));
    }

    SolidColor colorRequest = new SolidColor(0, 399);

    public Command getSetColor(Color color) {
        return run(() -> candle.setControl(colorRequest.withColor(new RGBWColor(color))));
    }
}
