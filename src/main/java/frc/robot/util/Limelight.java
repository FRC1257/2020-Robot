package frc.robot.util;

import static frc.robot.Constants.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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
        if(table.getEntry("tv").getDouble(0) == 1 && table.getEntry("tv").getDouble(1) == 1 && table.getEntry("tv").getDouble(2) == 1)
            return true;
        else{
            return false;
        }
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
}