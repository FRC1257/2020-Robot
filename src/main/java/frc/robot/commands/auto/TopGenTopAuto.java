package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TrajectoryLoader;

public class TopGenTopAuto extends SequentialCommandGroup {

    private final Drivetrain drivetrain;

    public TopGenTopAuto(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        Trajectory topGenTop1 = TrajectoryLoader.loadTrajectoryFromFile("Top-Gen-Top-1.wpilib.json");
        Trajectory topGenTop2 = TrajectoryLoader.loadTrajectoryFromFile("Top-Gen-Top-2.wpilib.json");

        addCommands(new ResetAutoPositionCommand(drivetrain, TrajectoryLoader.getInitialPoseReversed(topGenTop1)),
            new DriveTrajectoryCommand(drivetrain, topGenTop1, true), 
            new DriveTrajectoryCommand(drivetrain, topGenTop2, false));
    }
}