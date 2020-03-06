package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.SEC_PER_M;

// Use with .withTimeout() to add a timeout
public class DriveDistanceCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final double dist;
    private final Timer timer;

    public DriveDistanceCommand(Drivetrain drivetrain, double dist) {
        this.drivetrain = drivetrain;
        this.dist = dist;
        timer = new Timer();

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
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
        return drivetrain.getState() != Drivetrain.State.PID_DIST || timer.get() > SEC_PER_M * dist;
    }
}
