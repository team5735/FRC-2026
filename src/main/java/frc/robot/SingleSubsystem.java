package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.NTable;

public abstract class SingleSubsystem extends TimedRobot {
    protected final CommandXboxController controller = new CommandXboxController(0);

    protected SingleSubsystem() {
    }

    protected abstract void configureBindings();

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        NTable.updateAllSendables();
    }
}
