package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class ToggleElevatorLockCommand extends InstantCommand {

    public final Elevator elevator;

    public ToggleElevatorLockCommand(Elevator elevator) {
        this.elevator = elevator;
        
        addRequirements(elevator);
    }

    public void initialize() {
        elevator.toggleLock();
    }
}
