package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TrajectoryLoader;

public class BottomAuto extends SequentialCommandGroup {

    private final Drivetrain drivetrain;

    public BottomAuto(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        Trajectory bottomTrajectory = TrajectoryLoader.loadTrajectoryFromFile("Bottom.wpilib.json");

        addCommands(new ResetAutoPositionCommand(drivetrain, TrajectoryLoader.getInitialPoseReversed(bottomTrajectory)), 
            new DriveTrajectoryCommand(drivetrain, bottomTrajectory, true));
    }
}
