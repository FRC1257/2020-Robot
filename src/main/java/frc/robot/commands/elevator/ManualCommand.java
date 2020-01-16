package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ManualCommand extends CommandBase {

    private final Elevator elevator;
    private final XboxController controller;

    public ManualCommand(Elevator elevator, XboxController controller) {
        this.elevator = elevator;
        this.controller = controller;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.setElevatorSpeed(controller.getY(Hand.kRight)); // TODO maybe change, idk how we're programming the controllers
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}