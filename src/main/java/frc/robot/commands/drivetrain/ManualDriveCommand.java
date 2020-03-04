package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ManualDriveCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier turnSupplier;
    private final BooleanSupplier visionSupplier;
    private final boolean useVision;

    public ManualDriveCommand(Drivetrain drivetrain, DoubleSupplier forwardSupplier, DoubleSupplier turnSupplier,
        BooleanSupplier visionSupplier, boolean useVision) {
        this.drivetrain = drivetrain;
        this.forwardSupplier = forwardSupplier;
        this.turnSupplier = turnSupplier;
        this.visionSupplier = visionSupplier;
        this.useVision = useVision;

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
        drivetrain.manualDrive(forwardSupplier.getAsDouble(), turnSupplier.getAsDouble() + visionAdd);
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
