package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TrajectoryLoader;

public class TopGenTopAuto extends SequentialCommandGroup {

    public TopGenTopAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {
        
        Trajectory topGenTop1 = TrajectoryLoader.loadTrajectoryFromFile("Top-Gen-Top-1.wpilib.json");
        Trajectory topGenTop2 = TrajectoryLoader.loadTrajectoryFromFile("Top-Gen-Top-2.wpilib.json");

        addCommands(new TrajDriveAndShoot(drivetrain, indexer, shooter, topGenTop1, intake), 
            new DriveTrajectoryCommand(drivetrain, topGenTop2, false));
    }
}