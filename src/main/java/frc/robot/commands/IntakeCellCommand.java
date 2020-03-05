package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;

/**
 * Intakes the power cells into the intake and raise the indexer while available
 */
public class IntakeCellCommand extends CommandBase {

    private final Intake intake;
    private final Indexer indexer;
    private final BooleanSupplier booleanSupplier;

    public IntakeCellCommand(Intake intake, Indexer indexer, BooleanSupplier booleanSupplier) {
        this.intake = intake;
        this.indexer = indexer;
        this.booleanSupplier = booleanSupplier;

        addRequirements(intake, indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.intake();
        indexer.setOverride(booleanSupplier.getAsBoolean());
        indexer.raise();
    }

    @Override
    public void end(boolean interrupted) {
        intake.neutral();
        indexer.neutral();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}