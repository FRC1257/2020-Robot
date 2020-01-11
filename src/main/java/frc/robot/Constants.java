package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static int CONTROLLER_ID = 0;

    public static double DRIVE_TRACK_WIDTH_M = 1.0; // m
    public static double DRIVE_WHEEL_DIAM_M = 0.1524; // m

    public static double DRIVE_LEFT_VEL_PID_P = 0.1;
    public static double DRIVE_RIGHT_VEL_PID_P = 0.1;
    public static double DRIVE_MAX_VEL = 3.0; // m/s
    public static double DRIVE_MAX_ROT = 1.0; // rad/s

    public static double DRIVE_DIST_PID_P = 0.1;
    public static double DRIVE_DIST_PID_I = 0.0;
    public static double DRIVE_DIST_PID_D = 0.0;
    public static double DRIVE_DIST_PID_TOLERANCE = 0.1;
    public static double DRIVE_MAINTAIN_ANGLE_PID_P = 0.1;

    public static double DRIVE_ANGLE_PID_P = 0.1;
    public static double DRIVE_ANGLE_PID_I = 0.0;
    public static double DRIVE_ANGLE_PID_D = 0.0;
    public static double DRIVE_ANGLE_PID_TOLERANCE = 0.1;

    public static double DRIVE_KS = 0.0;
    public static double DRIVE_KV = 0.0;
    public static double DRIVE_KA = 0.0;

    public static double DRIVE_PROFILE_MAX_VEL = 3.0; // m/s
    public static double DRIVE_PROFILE_MAX_ACC = 5.0; // m/s^2
    public static double DRIVE_PROFILE_RIGHT_POS_P = 0.1;
    public static double DRIVE_PROFILE_LEFT_POS_P = 0.1;

    public static double DRIVE_RAMSETE_B = 2.0;
    public static double DRIVE_RAMSETE_ZETA = 0.7;
    public static double DRIVE_RAMSETE_MAX_VEL = 3.0; // m/s
    public static double DRIVE_RAMSETE_MAX_ACC = 5.0; // m/s^2
    public static double DRIVE_MAX_VOLTAGE_RAMSETE = 10.0; // V

    public static int DRIVE_FRONT_LEFT = 7;
    public static int DRIVE_FRONT_RIGHT = 2;
    public static int DRIVE_BACK_LEFT = 6;
    public static int DRIVE_BACK_RIGHT = 1;
}
