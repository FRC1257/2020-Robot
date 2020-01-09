package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

// Use with .withTimeout() to add a timeout
public class TurnAngleCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final double angle;

    public TurnAngleCommand(Drivetrain drivetrain, double angle) {
        this.drivetrain = drivetrain;
        this.angle = angle;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.turnAngle(angle);
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
        return drivetrain.getState() != Drivetrain.State.PID_ANGLE;
    }
}
