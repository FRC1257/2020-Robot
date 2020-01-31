package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TrajectoryLoader;

public class MiddleGenTopAuto extends SequentialCommandGroup {

    private final Drivetrain drivetrain;

    public MiddleGenTopAuto(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        Trajectory midGenTop1 = TrajectoryLoader.loadTrajectoryFromFile("Middle-Gen-Top-1.wpilib.json");
        Trajectory midGenTop2 = TrajectoryLoader.loadTrajectoryFromFile("Middle-Gen-Top-2.wpilib.json");

        addCommands(new ResetAutoPositionCommand(drivetrain, midGenTop1.getInitialPose()), 
            new DriveTrajectoryCommand(drivetrain, midGenTop1),
            new DriveTrajectoryCommand(drivetrain, midGenTop2));

    }
}