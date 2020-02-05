package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ResetAutoPositionCommand extends InstantCommand {

    public ResetAutoPositionCommand(Drivetrain drivetrain, Pose2d pose) {
        drivetrain.setRobotPose(pose);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }
}