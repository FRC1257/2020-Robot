package frc.robot.commands.auto.hardcode;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.indexer.IndexerShootCommand;
import frc.robot.commands.intake.IntakeReleaseCommand;
import frc.robot.commands.shooter.ShooterPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.INDEXER_DUMP_TIME;
import static frc.robot.Constants.SERVO_RELEASE_TIME;;

public class PrepareAndShoot extends ParallelDeadlineGroup {

    public PrepareAndShoot(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {
        super(new IndexerShootCommand(indexer, shooter, () -> true).withTimeout(INDEXER_DUMP_TIME),
            new ShooterPIDCommand(shooter),
            new IntakeReleaseCommand(intake).withTimeout(SERVO_RELEASE_TIME)
        );
    }
}