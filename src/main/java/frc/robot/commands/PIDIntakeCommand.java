package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class PIDIntakeCommand extends Command {
    private double targetPosition;
    private IntakeSubsystem intake;


    public PIDIntakeCommand(double targetPosition, IntakeSubsystem intake) {
        this.intake = intake;
        this.targetPosition = targetPosition;
    }

    @Override
    public void initialize() {
        intake.setGoal(targetPosition);
    }
    
    @Override
    public void execute() {
        intake.usePID();
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
