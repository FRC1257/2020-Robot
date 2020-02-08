package frc.robot.commands.auto;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.commands.indexer.IndexerShootCommand;
import frc.robot.commands.shooter.ShooterPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TrajectoryLoader;

public class DriveWhileShoot extends ParallelDeadlineGroup {

    public DriveWhileShoot(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Trajectory trajectory) {
        super(new SequentialCommandGroup(
                new ResetAutoPositionCommand(drivetrain, TrajectoryLoader.getInitialPoseReversed(trajectory)),
                new DriveTrajectoryCommand(drivetrain, trajectory, true),
                (new IndexerShootCommand(indexer)).withTimeout(INDEXER_DUMP_TIME)),
            new ShooterPIDCommand(shooter)
        );
    }
}