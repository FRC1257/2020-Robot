package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import static frc.robot.Constants.*;

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
        indexer.moveOneIndex();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.neutral();
    }

    @Override
    public boolean isFinished() {
        return (!(Math.abs(getPIDSetpoint() - indexer.getEncoderVal()) <= INDEXER_DEADBAND));
    }
}