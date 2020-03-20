package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class SetRobotPoseCommand extends InstantCommand {

    private Drivetrain drivetrain;
    private Pose2d pose;

    public SetRobotPoseCommand(Drivetrain drivetrain, Pose2d pose) {
        this.drivetrain = drivetrain;
        this.pose = pose;
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setRobotPose(pose);
    }
}