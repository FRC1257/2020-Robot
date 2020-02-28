package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * Intakes the power cells from the intake and shoots all balls in the indexer
 */
public class ShootCellCommand extends CommandBase {

    private final Intake intake;
    private final Indexer indexer;

    public ShootCellCommand(Intake intake, Indexer indexer) {
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
        indexer.shoot();
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