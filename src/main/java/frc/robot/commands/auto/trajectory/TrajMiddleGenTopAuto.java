package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TrajMiddleGenTopAuto extends SequentialCommandGroup {

    public TrajMiddleGenTopAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {

        Trajectory midGenTop1 = Trajectories.getTrajectory("Middle-Gen-Top-1");
        Trajectory midGenTop2 = Trajectories.getTrajectory("Middle-Gen-Top-2");

        addCommands(new TrajDriveAndShoot(drivetrain, indexer, shooter, midGenTop1, intake),
            new DriveTrajectoryCommand(drivetrain, midGenTop2));
    }
}