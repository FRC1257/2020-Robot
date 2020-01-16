package frc.robot.commands.tian;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FunPlusPhoenix;

// Use with .withTimeout() to add a timeout
public class DriveDistanceCommand extends CommandBase {

    private final FunPlusPhoenix drivetrain;
    private final double dist;

    public DriveDistanceCommand(FunPlusPhoenix drivetrain, double dist) {
        this.drivetrain = drivetrain;
        this.dist = dist;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.driveDist(dist);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return drivetrain.getState() != FunPlusPhoenix.State.PID_DIST;
    }
}
