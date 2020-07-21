package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TrajMiddleGenBottomAuto extends SequentialCommandGroup {
    
    public TrajMiddleGenBottomAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {

        Trajectory midGenBottom1 = Trajectories.getTrajectory("Mid-Power");
        Trajectory midGenBottom2 = Trajectories.getTrajectory("Power-Gen-Bottom");

        addCommands(new TrajDriveAndShoot(drivetrain, indexer, shooter, midGenBottom1, intake),
            new DriveTrajectoryCommand(drivetrain, midGenBottom2));
    }
}