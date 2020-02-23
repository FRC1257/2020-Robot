package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTrajectoryCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final Trajectory trajectory;
    private final boolean reversed;

    public DriveTrajectoryCommand(Drivetrain drivetrain, Trajectory trajectory, boolean reversed) {
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
        this.reversed = reversed;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.driveTrajectory(trajectory, reversed);
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
        return drivetrain.getState() != Drivetrain.State.RAMSETE;
    }
}
