package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IndexerEjectCommand extends CommandBase {

    private final Intake indexer;

    public IndexerEjectCommand(Intake indexer) {
        this.indexer = indexer;        
        addRequirements(indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        indexer.eject();
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