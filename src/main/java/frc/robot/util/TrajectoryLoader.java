package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import java.io.IOException;
import java.nio.file.Path;

import static frc.robot.Constants.PI;

public class TrajectoryLoader {

    public static Trajectory loadTrajectoryFromFile(String filename) {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + filename);
        Trajectory trajectory = null;

        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory: " + filename, e.getStackTrace());
        }

        return trajectory;
    }

    public static Pose2d getInitialPoseReversed(Trajectory trajectory) {
        return trajectory.getInitialPose().transformBy(new Transform2d(new Translation2d(0, 0), new Rotation2d(PI)));
    }
}
