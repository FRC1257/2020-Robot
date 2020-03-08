package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

/**
 * - Moves the power cells to the shooter via the conveyor belt 
 * 
 * - The stop mechanism allows the them to the enter the shooter
 */
public class IndexerShootCommand extends CommandBase {

    private final Indexer indexer;
    private final Shooter shooter;
    private final BooleanSupplier override;

    public IndexerShootCommand(Indexer indexer, Shooter shooter, BooleanSupplier override) {
        this.indexer = indexer;
        this.shooter = shooter;
        this.override = override;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (shooter.withinTolerance() || override.getAsBoolean()) {
            indexer.shoot();
        }
        else {
            indexer.neutral();
        }
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
