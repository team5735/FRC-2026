package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRollerCommand extends Command {
    
    private IntakeSubsystem intake;    

    public IntakeRollerCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
      intake.intake_run();
    }
    
    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        intake.intake_stop();
    }

    @Override
    public boolean isFinished() {
      return false;
    }    
}
