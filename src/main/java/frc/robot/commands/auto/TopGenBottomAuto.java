package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TrajectoryLoader;

public class TopGenBottomAuto extends SequentialCommandGroup {

    public TopGenBottomAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {
        
        Trajectory topGenBottom1 = TrajectoryLoader.loadTrajectoryFromFile("Top-Gen-Bottom-1.wpilib.json");
        Trajectory topGenBottom2 = TrajectoryLoader.loadTrajectoryFromFile("Top-Gen-Bottom-2.wpilib.json");

        addCommands(new DriveWhileShoot(drivetrain, indexer, shooter, topGenBottom1, intake),
            new DriveTrajectoryCommand(drivetrain, topGenBottom2, false));
    }
}