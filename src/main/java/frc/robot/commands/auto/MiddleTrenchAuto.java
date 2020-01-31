package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TrajectoryLoader;

public class MiddleTrenchAuto extends SequentialCommandGroup {

    private final Drivetrain drivetrain;

    public MiddleTrenchAuto(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        Trajectory midTrench1 = TrajectoryLoader.loadTrajectoryFromFile("Middle-Trench-1.wpilib.json");
        Trajectory midTrench2 = TrajectoryLoader.loadTrajectoryFromFile("Middle-Trench-2.wpilib.json");

        addCommands(new ResetAutoPositionCommand(drivetrain, TrajectoryLoader.getInitialPoseReversed(midTrench1)),
            new DriveTrajectoryCommand(drivetrain, midTrench1, true),
            new DriveTrajectoryCommand(drivetrain, midTrench2, false));
    }

}