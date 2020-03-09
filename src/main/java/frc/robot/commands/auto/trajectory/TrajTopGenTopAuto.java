package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TrajTopGenTopAuto extends SequentialCommandGroup {

    public TrajTopGenTopAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {

        Trajectory topGenTop1 = Trajectories.getTrajectory("Top-Gen-Top-1");
        Trajectory topGenTop2 = Trajectories.getTrajectory("Top-Gen-Top-2");

        addCommands(new TrajDriveAndShoot(drivetrain, indexer, shooter, topGenTop1, intake), 
            new DriveTrajectoryCommand(drivetrain, topGenTop2));
    }
}