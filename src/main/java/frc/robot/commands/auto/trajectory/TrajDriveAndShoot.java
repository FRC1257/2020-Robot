package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.commands.indexer.IndexerShootCommand;
import frc.robot.commands.shooter.ShooterPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.Autonomous.INDEXER_DUMP_TIME;

public class TrajDriveAndShoot extends ParallelDeadlineGroup {

    public TrajDriveAndShoot(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Trajectory trajectory, Intake intake) {
        super(new SequentialCommandGroup(
                new SetRobotPoseCommand(drivetrain, trajectory.getInitialPose()),
                new DriveTrajectoryCommand(drivetrain, trajectory),
                (new IndexerShootCommand(indexer, shooter, () -> true)).withTimeout(INDEXER_DUMP_TIME)),
            new ShooterPIDCommand(shooter)
        );
    }
}