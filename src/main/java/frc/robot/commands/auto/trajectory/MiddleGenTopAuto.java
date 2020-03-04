package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TrajectoryLoader;

public class MiddleGenTopAuto extends SequentialCommandGroup {

    public MiddleGenTopAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {
        
        Trajectory midGenTop1 = TrajectoryLoader.loadTrajectoryFromFile("Middle-Gen-Top-1.wpilib.json");
        Trajectory midGenTop2 = TrajectoryLoader.loadTrajectoryFromFile("Middle-Gen-Top-2.wpilib.json");

        addCommands(new TrajDriveAndShoot(drivetrain, indexer, shooter, midGenTop1, intake),
            new DriveTrajectoryCommand(drivetrain, midGenTop2, false));
    }
}