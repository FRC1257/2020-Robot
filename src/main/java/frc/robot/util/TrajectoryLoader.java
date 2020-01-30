package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

public class TrajectoryLoader {

    public static Trajectory loadTrajectoryFromFile(String filename) {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + filename);
        Trajectory trajectory = null;

        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        }
        catch(IOException e) {
            DriverStation.reportError("Unable to open trajectory: " + filename, e.getStackTrace());
        }

        return trajectory;
    }
}