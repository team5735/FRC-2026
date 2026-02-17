package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HoodSubsystem;

public class MoveCommand extends InstantCommand {

    public MoveCommand(HoodSubsystem subsystem, double position) {super(() -> subsystem.setPosition(position), subsystem);
    }
}