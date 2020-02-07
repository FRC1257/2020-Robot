package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    
    public static int CONTROLLER_DRIVE_ID = 0;
    public static int CONTROLLER_OPERATOR_ID = 1;

    public final static int NEO_CURRENT_LIMIT = 80;

    public final static int ELEVATOR_MOTOR_ID = 0;

    public static final double[] ELEVATOR_PIDF = {0, 1, 2, 3};
    public static final double ELEVATOR_PID_MAX_OUTPUT = 1;
    public static final double ELEVATOR_PID_MIN_OUTPUT = -1;

    public static final double ELEVATOR_TOP = 100;
}