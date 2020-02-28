package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * Intakes the power cells into the intake and raise the indexer while available
 */
public class IntakeCellCommand extends CommandBase {

    private final Intake intake;
    private final Indexer indexer;

    public IntakeCellCommand(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;

        addRequirements(intake, indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.intake();
        
        if (indexer.canMove()) {
            indexer.raise();
        }
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