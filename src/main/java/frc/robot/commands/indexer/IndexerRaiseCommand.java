package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/**
 * Moves the power cells towards the shooter via the conveyor belt
 */
public class IndexerRaiseCommand extends CommandBase {

    private final Indexer indexer;
    private final DoubleSupplier topSpeedSupplier;

    public IndexerRaiseCommand(Indexer indexer, DoubleSupplier topSpeedSupplier) {
        this.indexer = indexer;
        this.topSpeedSupplier = topSpeedSupplier;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        indexer.raise();
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
