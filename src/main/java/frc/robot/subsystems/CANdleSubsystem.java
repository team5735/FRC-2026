package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.PartialRobot;
import frc.robot.constants.Constants;
import frc.robot.util.MatchState;

public class CANdleSubsystem extends SubsystemBase {

    private final CANdle candle = new CANdle(Constants.CANDLE_ID);

    private final SolidColor colorRequest = new SolidColor(0, 399);

    public final Trigger isAllianceHubActiveTrigger = new Trigger(this::isAllianceHubActive);

    public CANdleSubsystem() {
        candle.getConfigurator().apply(
            new CANdleConfiguration().withLED(
                new LEDConfigs().withStripType(StripTypeValue.GRB)
                .withBrightnessScalar(0.5))
            );
    }

    public Command setColor(Color color) {
        return runOnce(() -> 
            candle.setControl(
                colorRequest.withColor(new RGBWColor(color))
            )
        );
    }

    public Boolean isAllianceHubActive() {
        var alliance = DriverStation.getAlliance();
        return MatchState.isHubActive(alliance);
    }

    // This is a full robot config for testing the CANdle subsystem and the LEDs
    public static class Tester extends PartialRobot {
        
    public CANdleSubsystem CANdle = new CANdleSubsystem();

        public Tester() {
            super();

            controller.x().onTrue(CANdle.setColor(Color.kGreen)); // This is the custom color for green
            controller.a().onTrue(CANdle.setColor(Color.kRed));              
            controller.b().onTrue(CANdle.setColor(Color.kBlue)); 
            controller.y().onTrue(CANdle.setColor(new Color(0.0, 0.0, 0.0))); // This is the custom color for off

            CANdle.isAllianceHubActiveTrigger.onTrue(Commands.runOnce(() -> {
                SmartDashboard.putBoolean("CANdle/hub_active", true);
                CANdle.setColor(Color.kRed);
            }));
            CANdle.isAllianceHubActiveTrigger.onFalse(Commands.runOnce(() -> {
                SmartDashboard.putBoolean("CANdle/hub_active", false);
                CANdle.setColor(new Color(0.0, 0.0, 0.0)); //Custom color for black
            }));

        }

        @Override
        public void robotPeriodic() {
            super.robotPeriodic();
        }

    };
}
