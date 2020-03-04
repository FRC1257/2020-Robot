package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TrajectoryLoader;

public class MiddleGenBottomAuto extends SequentialCommandGroup {
    
    public MiddleGenBottomAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {
        
        Trajectory midGenBottom1 = TrajectoryLoader.loadTrajectoryFromFile("Middle-Gen-Bottom-1.wpilib.json");
        Trajectory midGenBottom2 = TrajectoryLoader.loadTrajectoryFromFile("Middle-Gen-Bottom-2.wpilib.json");

        addCommands(new TrajDriveAndShoot(drivetrain, indexer, shooter, midGenBottom1, intake),
            new DriveTrajectoryCommand(drivetrain, midGenBottom2, false));
    }
}