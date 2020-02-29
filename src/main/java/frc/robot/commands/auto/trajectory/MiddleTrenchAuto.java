package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TrajectoryLoader;

public class MiddleTrenchAuto extends SequentialCommandGroup {

    public MiddleTrenchAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {

        Trajectory midTrench1 = TrajectoryLoader.loadTrajectoryFromFile("Middle-Trench-1.wpilib.json");
        Trajectory midTrench2 = TrajectoryLoader.loadTrajectoryFromFile("Middle-Trench-2.wpilib.json");

        addCommands(new DriveWhileShoot(drivetrain, indexer, shooter, midTrench1, intake),
            new DriveTrajectoryCommand(drivetrain, midTrench2, false));
    }

}