package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ELEVATOR_MAX_VEL;

public class ClosedLoopElevatorCommand extends CommandBase {

    private Elevator elevator;
    private DoubleSupplier speedSupplier;

    public ClosedLoopElevatorCommand(Elevator elevator, DoubleSupplier speedSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        elevator.setElevatorSpeedClosedLoop(speedSupplier.getAsDouble() * ELEVATOR_MAX_VEL);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}