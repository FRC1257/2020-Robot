package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * Ejects the power cells from the intake and lowers the indexer
 */
public class EjectCellCommand extends CommandBase {

    private final Intake intake;
    private final Indexer indexer;

    public EjectCellCommand(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;

        addRequirements(intake, indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.eject();
        indexer.lower();
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