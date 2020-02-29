package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.commands.indexer.IndexerShootCommand;
import frc.robot.commands.intake.IntakeReleaseCommand;
import frc.robot.commands.shooter.ShooterPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TrajectoryLoader;

import static frc.robot.Constants.INDEXER_DUMP_TIME;
import static frc.robot.Constants.SERVO_RELEASE_TIME;

public class DriveWhileShoot extends ParallelDeadlineGroup {

    public DriveWhileShoot(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Trajectory trajectory, Intake intake) {
        super(new SequentialCommandGroup(
                new ResetAutoPositionCommand(drivetrain, TrajectoryLoader.getInitialPoseReversed(trajectory)),
                new DriveTrajectoryCommand(drivetrain, trajectory, true),
                (new IndexerShootCommand(indexer, shooter, () -> true)).withTimeout(INDEXER_DUMP_TIME)),
            new ShooterPIDCommand(shooter),
            new IntakeReleaseCommand(intake).withTimeout(SERVO_RELEASE_TIME)
        );
    }
}