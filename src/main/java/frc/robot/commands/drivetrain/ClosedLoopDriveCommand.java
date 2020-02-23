package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.DRIVE_MAX_ROT;
import static frc.robot.Constants.DRIVE_MAX_VEL;

public class ClosedLoopDriveCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier turnSupplier;

    public ClosedLoopDriveCommand(Drivetrain drivetrain, DoubleSupplier forwardSupplier, DoubleSupplier turnSupplier) {
        this.drivetrain = drivetrain;
        this.forwardSupplier = forwardSupplier;
        this.turnSupplier = turnSupplier;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.closedLoopDrive(forwardSupplier.getAsDouble() * DRIVE_MAX_VEL,
                turnSupplier.getAsDouble()* DRIVE_MAX_ROT);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
