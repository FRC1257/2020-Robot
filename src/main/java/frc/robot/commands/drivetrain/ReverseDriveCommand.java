package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ReverseDriveCommand extends InstantCommand {

    private Drivetrain drivetrain;

    public ReverseDriveCommand(Drivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.toggleReverse();
    }
}
