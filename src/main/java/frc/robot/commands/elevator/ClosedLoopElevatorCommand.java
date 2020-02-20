package frc.robot.commands.elevator;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ClosedLoopElevatorCommand extends CommandBase {

    private Elevator elevator;
    private XboxController controller;

    public ClosedLoopElevatorCommand(Elevator elevator, XboxController controller) {
        this.elevator = elevator;
        this.controller = controller;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        elevator.setElevatorSpeedClosedLoop((controller.getY(Hand.kLeft)) * ELEVATOR_VEL_PID_KP);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}