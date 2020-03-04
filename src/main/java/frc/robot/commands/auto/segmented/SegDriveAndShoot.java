package frc.robot.commands.auto.segmented;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveDistanceCommand;
import frc.robot.commands.indexer.IndexerShootCommand;
import frc.robot.commands.intake.IntakeReleaseCommand;
import frc.robot.commands.shooter.ShooterPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.INDEXER_DUMP_TIME;
import static frc.robot.Constants.SERVO_RELEASE_TIME;
import static frc.robot.Constants.AUTO_WALL_FLUSH_TIME;

public class SegDriveAndShoot extends ParallelDeadlineGroup {

    public SegDriveAndShoot(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake, double dist) {
        super(new SequentialCommandGroup(
                new DriveDistanceCommand(drivetrain, dist).withTimeout(AUTO_WALL_FLUSH_TIME),
                (new IndexerShootCommand(indexer, shooter, () -> true)).withTimeout(INDEXER_DUMP_TIME)),
            new ShooterPIDCommand(shooter),
            new IntakeReleaseCommand(intake).withTimeout(SERVO_RELEASE_TIME)
        );
    }
}
