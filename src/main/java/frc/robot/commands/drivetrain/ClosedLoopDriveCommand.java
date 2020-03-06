package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.DRIVE_MAX_ROT;
import static frc.robot.Constants.DRIVE_MAX_VEL;

public class ClosedLoopDriveCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier turnSupplier;
    private final BooleanSupplier visionSupplier;
    private final boolean useVision;
    private final SlewRateLimiter limiter;

    public ClosedLoopDriveCommand(Drivetrain drivetrain, DoubleSupplier forwardSupplier, DoubleSupplier turnSupplier,
        BooleanSupplier visionSupplier, boolean useVision) {
        this.drivetrain = drivetrain;
        this.forwardSupplier = forwardSupplier;
        this.turnSupplier = turnSupplier;
        this.visionSupplier = visionSupplier;
        this.useVision = useVision;
        this.limiter = new SlewRateLimiter(1.25);

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double visionAdd = 0;
        if (useVision && visionSupplier.getAsBoolean()) {
            visionAdd = Limelight.getVisionAdd();
        }

        double rot = turnSupplier.getAsDouble() + visionAdd;
        if (rot > 1.0) rot = 1.0;
        if (rot < -1.0) rot = -1.0;

        drivetrain.closedLoopDrive(limiter.calculate(forwardSupplier.getAsDouble()) * DRIVE_MAX_VEL,
            -rot * DRIVE_MAX_ROT);
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
