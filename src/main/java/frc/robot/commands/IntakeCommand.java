package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    
    private IntakeSubsystem i;    

    public IntakeCommand(IntakeSubsystem i) {
        this.i = i;
    }

    @Override
    public void initialize() {
      i.intake_run();
    }
    
    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        i.intake_stop();
    }

    @Override
    public boolean isFinished() {
      return false;
    }    
}
