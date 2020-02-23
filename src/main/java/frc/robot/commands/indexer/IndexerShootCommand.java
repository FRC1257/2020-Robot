package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/**
 * - Moves the power cells to the shooter via the conveyor belt 
 * 
 * - The stop mechanism allows the them to the enter the shooter
 */
public class IndexerShootCommand extends CommandBase {

    private final Indexer indexer;

    public IndexerShootCommand(Indexer indexer) {
        this.indexer = indexer;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        indexer.shoot();
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
