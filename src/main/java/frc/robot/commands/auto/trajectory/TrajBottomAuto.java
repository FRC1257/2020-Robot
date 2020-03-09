package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TrajBottomAuto extends SequentialCommandGroup {

    public TrajBottomAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {
        
        Trajectory bottomTrajectory = Trajectories.getTrajectory("Bottom");

        addCommands(
            new TrajDriveAndShoot(drivetrain, indexer, shooter, bottomTrajectory, intake));
    }
}
