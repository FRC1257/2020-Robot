package frc.robot.commands.auto.trajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static frc.robot.Constants.Drivetrain.DRIVE_PATH_MAX_VEL;
import static frc.robot.Constants.Drivetrain.DRIVE_PATH_MAX_ACC;
import static frc.robot.Constants.Drivetrain.DRIVE_TRACK_WIDTH_M;

public class Trajectories {

    private static class TrajectoryPoints {
        Pose2d init;
        Pose2d end;
        Translation2d[] intermediate;
        boolean reversed;

        private TrajectoryPoints(boolean reversed, double initHeading, double endHeading, Translation2d... translations) {
            this.reversed = reversed;
            this.init = new Pose2d(translations[0], Rotation2d.fromDegrees(initHeading));
            this.end = new Pose2d(translations[translations.length - 1], Rotation2d.fromDegrees(endHeading));
            this.intermediate = Arrays.copyOfRange(translations, 1, translations.length - 1);
        }
    }

    private static Map<String, TrajectoryPoints> pointMap = new HashMap<>();
    private static Map<String, Trajectory> trajectoryMap = new HashMap<>();
    private static TrajectoryConfig trajectoryConfig;

    public static void setUpTrajectories() {
        trajectoryConfig = new TrajectoryConfig(DRIVE_PATH_MAX_VEL, DRIVE_PATH_MAX_ACC);
        trajectoryConfig.setKinematics(new DifferentialDriveKinematics(DRIVE_TRACK_WIDTH_M));

        pointMap.put("Top-Power", new TrajectoryPoints(true, 0, 0,
            new Translation2d(3.048, -2.404), new Translation2d(0.445, -2.404)));
        pointMap.put("Mid-Power", new TrajectoryPoints(true, 0, 0, 
            new Translation2d(3.048, -4.105), new Translation2d(0.445, -2.404)));
        pointMap.put("Bot-Power", new TrajectoryPoints(true, 0, 0,
            new Translation2d(3.048, -5.655), new Translation2d(0.445, -2.404)));

        pointMap.put("Power-Trench", new TrajectoryPoints(false, 0, 0,
            new Translation2d(0.445, -2.404), new Translation2d(5.248, -0.705)));
        pointMap.put("Power-Gen-Top", new TrajectoryPoints(false, 0, 0,
            new Translation2d(0.445, -2.404), new Translation2d(5.247, -2.404)));
        pointMap.put("Power-Gen-Bot", new TrajectoryPoints(false, 0, 0,
            new Translation2d(0.445, -2.404), new Translation2d(5.29, -4.572)));

        for (String key : pointMap.keySet()) {
            generateTrajectory(key);
        }
    }

    public static Trajectory getTrajectory(String name) {
        return trajectoryMap.getOrDefault(name, null);
    }

    private static void generateTrajectory(String name) {
        if (trajectoryMap.containsKey(name)) return;

        TrajectoryPoints points = pointMap.get(name);
        trajectoryMap.put(name, TrajectoryGenerator.generateTrajectory(points.init, List.of(points.intermediate),
                points.end, trajectoryConfig.setReversed(points.reversed)));
    }
}
