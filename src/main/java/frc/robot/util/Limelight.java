package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.Vision.VISION_FEEDFORWARD;
import static frc.robot.Constants.Vision.VISION_KP;

public class Limelight {

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public static double getVisionAdd() {
        double visionAdd = 0.0;

        if (isTargetValid()) { 
            visionAdd = Math.copySign(VISION_FEEDFORWARD, getTargetX());
            visionAdd += VISION_KP * getTargetX();
        }

        return visionAdd;
    }

    public static boolean isTargetValid() {
        return table.getEntry("tv").getDouble(0) == 1 &&
                table.getEntry("tv").getDouble(1) == 1 &&
                table.getEntry("tv").getDouble(2) == 1;
    }

    public static double getTargetX() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public static double getTargetY() {
        return table.getEntry("ty").getDouble(0.0);
    }

    public static double getTargetArea() {
        return table.getEntry("ta").getDouble(0.0);
    }

    public static void tuningInit() {
        SmartDashboard.putNumber("Vision Feedforward", VISION_FEEDFORWARD);
        SmartDashboard.putNumber("Vision kP", VISION_KP);
    }

    public static void tuningPeriodic() {
        VISION_FEEDFORWARD = SmartDashboard.getNumber("Vision Feedforward", VISION_FEEDFORWARD);
        VISION_KP = SmartDashboard.getNumber("Vision kP", VISION_KP);
    }
}