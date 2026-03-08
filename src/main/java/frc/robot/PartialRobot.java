package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.NTable;

public abstract class PartialRobot extends TimedRobot {
    protected final CommandXboxController controller = new CommandXboxController(0);

    protected PartialRobot() {
        NTable.root().set("scheduler", CommandScheduler.getInstance());
        SignalLogger.enableAutoLogging(false);
        StatusLogger.disableAutoLogging();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        NTable.updateAllSendables();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
