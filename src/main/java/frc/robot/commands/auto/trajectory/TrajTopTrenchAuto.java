package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TrajTopTrenchAuto extends SequentialCommandGroup {

    public TrajTopTrenchAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {

        Trajectory topTrench1 = Trajectories.getTrajectory("Top-Trench-1");
        Trajectory topTrench2 = Trajectories.getTrajectory("Top-Trench-2");

        addCommands(new TrajDriveAndShoot(drivetrain, indexer, shooter, topTrench1, intake), 
            new DriveTrajectoryCommand(drivetrain, topTrench2));
    }

}