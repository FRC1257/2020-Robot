package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ReverseDriveCommand extends InstantCommand {

    private final Drivetrain drivetrain;

    public ReverseDriveCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        drivetrain.toggleReverse();
    }
}
