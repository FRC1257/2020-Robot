package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TrajMiddleTrenchAuto extends SequentialCommandGroup {

    public TrajMiddleTrenchAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {

        Trajectory midTrench1 = Trajectories.getTrajectory("Middle-Power");
        Trajectory midTrench2 = Trajectories.getTrajectory("Power-Trench");

        addCommands(new TrajDriveAndShoot(drivetrain, indexer, shooter, midTrench1, intake),
            new DriveTrajectoryCommand(drivetrain, midTrench2));
    }

}