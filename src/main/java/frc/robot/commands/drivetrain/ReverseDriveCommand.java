package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ReverseDriveCommand extends CommandBase {

    private final Drivetrain drivetrain;

    public ReverseDriveCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        drivetrain.toggleReverse();
    }
}
