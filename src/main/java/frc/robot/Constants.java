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
    
    // Xbox Controller Constants
    public static int CONTROLLER_DRIVER_ID = 0;
    public static int CONTROLLER_OPERATOR_ID = 1;

    // Intake Constants
    public static int INTAKE_MOTOR_ID = 1;

    public static double INTAKE_NEUTRAL_SPEED = 0.0;
    public static double INTAKE_INTAKE_SPEED = -0.8;
    public static double INTAKE_EJECT_SPEED = 0.8;

    // Indexer Constants
    public static int INDEXER_CONVEYOR_TOP_MOTOR_ID = 2;
    public static int INDEXER_CONVEYOR_BOTTOM_MOTOR_ID = 3;
    public static int INDEXER_STOP_MOTOR_ID = 4;

    public static double INDEXER_CONVEYOR_NEUTRAL_SPEED = 0.0;
    public static double INDEXER_CONVEYOR_SHOOT_SPEED = 0.5;
    public static double INDEXER_CONVEYOR_EJECT_SPEED = -0.5;
    public static double INDEXER_CONVEYOR_INTAKE_SPEED = 0.5;

    public static double INDEXER_STOP_NEUTRAL_SPEED = 0.0;
    public static double INDEXER_STOP_SHOOT_SPEED = 1.0;

    public static double INDEXER_DEADBAND = 5.0;
    public static double [] INDEXER_PIDF = {4,6,8,9};

    public static double RESET_SETPOINT = 0.0;
    public static double ONE_INDEX_SETPOINT = 90.0;

    public static double INDEXER_MAX_SPEED;

    //NEO Constants

    public static int NEO_550_CURRENT_LIMIT = 25;

    // Elevator Constants
    public final static int ELEVATOR_MOTOR_ID = 0;
}