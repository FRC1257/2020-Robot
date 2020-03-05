package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ManualElevatorCommand extends CommandBase {

    private final Elevator elevator;
    private final DoubleSupplier speedSupplier;
    private final BooleanSupplier overrideSupplier;

    public ManualElevatorCommand(Elevator elevator, DoubleSupplier speedSupplier, BooleanSupplier overrideSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
        this.overrideSupplier = overrideSupplier;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.setOverride(overrideSupplier.getAsBoolean());
        elevator.setElevatorSpeed(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}