package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/**
 * Moves the power cells to the intake via the conveyor belt
 */

public class IndexerLowerCommand extends CommandBase {

    private final Indexer indexer;

    public IndexerLowerCommand(Indexer indexer) {
        this.indexer = indexer;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        indexer.lower();
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