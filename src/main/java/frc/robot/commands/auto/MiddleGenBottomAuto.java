package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TrajectoryLoader;

public class MiddleGenBottomAuto extends SequentialCommandGroup {

    private final Drivetrain drivetrain;

    public MiddleGenBottomAuto(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        Trajectory midGenBottom1 = TrajectoryLoader.loadTrajectoryFromFile("Middle-Gen-Bottom-1.wpilib.json");
        Trajectory midGenBottom2 = TrajectoryLoader.loadTrajectoryFromFile("Middle-Gen-Bottom-2.wpilib.json");

        addCommands(new ResetAutoPositionCommand(drivetrain, midGenBottom1.getInitialPose()),
            new DriveTrajectoryCommand(drivetrain, midGenBottom1),
            new DriveTrajectoryCommand(drivetrain, midGenBottom2));
    }
}