package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TrajectoryLoader;

public class TopTrenchAuto extends SequentialCommandGroup {

    private final Drivetrain drivetrain;

    public TopTrenchAuto(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        Trajectory topTrench1 = TrajectoryLoader.loadTrajectoryFromFile("Top-Trench-1.wpilib.json");
        Trajectory topTrench2 = TrajectoryLoader.loadTrajectoryFromFile("Top-Trench-2.wpilib.json");

        addCommands(new ResetAutoPositionCommand(drivetrain, topTrench1.getInitialPose()),
            new DriveTrajectoryCommand(drivetrain, topTrench1), 
            new DriveTrajectoryCommand(drivetrain, topTrench2));
    }

}