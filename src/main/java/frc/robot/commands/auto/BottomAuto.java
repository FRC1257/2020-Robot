package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.util.TrajectoryLoader;

public class BottomAuto extends SequentialCommandGroup {

    public BottomAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter) {  
        
        Trajectory bottomTrajectory = TrajectoryLoader.loadTrajectoryFromFile("Bottom.wpilib.json");

        addCommands(
            new DriveWhileShoot(drivetrain, indexer, shooter, bottomTrajectory));
    }
}
