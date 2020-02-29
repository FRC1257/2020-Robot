package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TrajectoryLoader;

public class BottomAuto extends SequentialCommandGroup {

    public BottomAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {  
        
        Trajectory bottomTrajectory = TrajectoryLoader.loadTrajectoryFromFile("Bottom.wpilib.json");

        addCommands(
            new DriveWhileShoot(drivetrain, indexer, shooter, bottomTrajectory, intake));
    }
}
