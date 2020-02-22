package frc.robot.commands.elevator;

import static frc.robot.util.MathUtils.applyDeadband;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ManualElevatorCommand extends CommandBase {

    private final Elevator elevator;
    private final XboxController controller;

    public ManualElevatorCommand(Elevator elevator, XboxController controller) {
        this.elevator = elevator;
        this.controller = controller;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.setElevatorSpeed(applyDeadband(-controller.getY(Hand.kLeft)));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}