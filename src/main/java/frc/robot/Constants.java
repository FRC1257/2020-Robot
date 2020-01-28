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
    public static int INDEXER_CONVEYER_MOTOR_ID = 2;
    public static int INDEXER_STOP_MOTOR_ID = 3;

    public static double INDEXER_CONVEYER_NEUTRAL_SPEED = 0.0;
    public static double INDEXER_CONVEYER_SHOOT_SPEED = 0.5;
    public static double INDEXER_CONVEYER_EJECT_SPEED = -0.5;
    public static double INDEXER_CONVEYER_INTAKE_SPEED = 0.5;

    public static double INDEXER_STOP_NEUTRAL_SPEED = 0.0;
    public static double INDEXER_STOP_SHOOT_SPEED = 1.0;

    // Elevator Constants
    public final static int ELEVATOR_MOTOR_ID = 0;
}