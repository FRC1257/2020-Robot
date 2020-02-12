package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class ToggleLockCommand extends InstantCommand {

    public ToggleLockCommand(Elevator elevator) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevator);

        elevator.toggleLock();
    }
}
