package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TrajTopGenBottomAuto extends SequentialCommandGroup {

    public TrajTopGenBottomAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {

        Trajectory topGenBottom1 = Trajectories.getTrajectory("Top-Gen-Bottom-1");
        Trajectory topGenBottom2 = Trajectories.getTrajectory("Top-Gen-Bottom-2");

        addCommands(new TrajDriveAndShoot(drivetrain, indexer, shooter, topGenBottom1, intake),
            new DriveTrajectoryCommand(drivetrain, topGenBottom2));
    }
}