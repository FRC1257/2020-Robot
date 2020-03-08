package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

import java.util.function.DoubleSupplier;

/**
 * Maintains the position of the power cells in the indexer
 */
public class IndexerNeutralCommand extends CommandBase {

    private final Indexer indexer;
    private final DoubleSupplier topSpeedSupplier;

    public IndexerNeutralCommand(Indexer indexer, DoubleSupplier topSpeedSupplier) {
        this.indexer = indexer;
        this.topSpeedSupplier = topSpeedSupplier;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        indexer.neutral();
        indexer.setTopSpeed(topSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        indexer.neutral();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
