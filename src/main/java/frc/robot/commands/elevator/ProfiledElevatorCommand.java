package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ProfiledElevatorCommand extends CommandBase {

    private Elevator elevator;

    public ProfiledElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        elevator.raiseProfiled();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}