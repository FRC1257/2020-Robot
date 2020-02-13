package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/**
 * - Uses PID control to move the indexer to a specified amount
 * 
 * - PID control can be exited by activating another command
 */
public class IndexerPIDCommand extends CommandBase {

    private final Indexer indexer;

    public IndexerPIDCommand(Indexer indexer) {
        this.indexer = indexer;        
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        indexer.advance();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return indexer.getState() != Indexer.State.PID;
    }
}