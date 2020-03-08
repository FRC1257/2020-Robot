package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

import java.util.function.DoubleSupplier;

/**
 * Moves the power cells to the intake via the conveyor belt
 */

public class IndexerLowerCommand extends CommandBase {

    private final Indexer indexer;
    private final DoubleSupplier topSpeedSupplier;

    public IndexerLowerCommand(Indexer indexer, DoubleSupplier topSpeedSupplier) {
        this.indexer = indexer;
        this.topSpeedSupplier = topSpeedSupplier;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        indexer.lower();
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