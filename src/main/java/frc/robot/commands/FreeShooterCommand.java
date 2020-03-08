package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;


public class FreeShooterCommand extends CommandBase {

    private final Shooter shooter;
    private final Indexer indexer;

    public FreeShooterCommand(Shooter shooter, Indexer indexer) {
        this.shooter = shooter;
        this.indexer = indexer;

        addRequirements(shooter);
        addRequirements(indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.back();
        indexer.back();

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}