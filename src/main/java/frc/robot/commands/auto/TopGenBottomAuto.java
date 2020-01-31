package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TrajectoryLoader;

public class TopGenBottomAuto extends SequentialCommandGroup {

    private final Drivetrain drivetrain;

    public TopGenBottomAuto(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        Trajectory topGenBottom1 = TrajectoryLoader.loadTrajectoryFromFile("Top-Gen-Bottom-1.wpilib.json");
        Trajectory topGenBottom2 = TrajectoryLoader.loadTrajectoryFromFile("Top-Gen-Bottom-2.wpilib.json");

        addCommands(new ResetAutoPositionCommand(drivetrain, topGenBottom1.getInitialPose()),
            new DriveTrajectoryCommand(drivetrain, topGenBottom1),
            new DriveTrajectoryCommand(drivetrain, topGenBottom2));
    }
}