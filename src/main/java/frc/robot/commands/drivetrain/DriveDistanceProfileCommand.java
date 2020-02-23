package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistanceProfileCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final double dist;

    public DriveDistanceProfileCommand(Drivetrain drivetrain, double dist) {
        this.drivetrain = drivetrain;
        this.dist = dist;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.driveDistProfile(dist);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return drivetrain.getState() != Drivetrain.State.PROFILE_DIST;
    }
}
