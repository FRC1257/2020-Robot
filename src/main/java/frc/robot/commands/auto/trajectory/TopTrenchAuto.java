package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TrajectoryLoader;

public class TopTrenchAuto extends SequentialCommandGroup {

    public TopTrenchAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {

        Trajectory topTrench1 = TrajectoryLoader.loadTrajectoryFromFile("Top-Trench-1.wpilib.json");
        Trajectory topTrench2 = TrajectoryLoader.loadTrajectoryFromFile("Top-Trench-2.wpilib.json");

        addCommands(new TrajDriveAndShoot(drivetrain, indexer, shooter, topTrench1, intake), 
            new DriveTrajectoryCommand(drivetrain, topTrench2, true));
    }

}