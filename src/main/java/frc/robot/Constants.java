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

    // Xbox Controller Constants
    public static int CONTROLLER_DRIVER_ID = 0;
    public static int CONTROLLER_OPERATOR_ID = 1;

    // NEO Constants
    public static int NEO_CURRENT_LIMIT = 80;
    public static int NEO_550_CURRENT_LIMIT = 25;
    
    // Autonomous Constants
    public static double INDEXER_DUMP_TIME = 2.0; // seconds
    public static double SERVO_RELEASE_TIME = 5.0; // seconds

    public enum AutoPosition {
        TOP, 
        MIDDLE,
        BOTTOM
    }

    public enum AutoGoal {
        DEFAULT,
        TRENCH,
        GEN_BOTTOM,
        GEN_TOP
    }

    // Elevator Constants
    public final static int ELEVATOR_MOTOR_ID = 10;
    public final static int ELEVATOR_FOLLOWER_MOTOR_ID = 13;
    public final static int ELEVATOR_BRAKE_SERVO_ID = 1;

    public static double ELEVATOR_SETPOINT = 100;
    public final static double ELEVATOR_BRAKE_POSITION = 1.0;
    public final static double ELEVATOR_MAX_HEIGHT = 105;
    
    public static double ELEVATOR_KS = 0.0;
    public static double ELEVATOR_KG = 0.0;
    public static double ELEVATOR_KV = 0.0;
    public static double ELEVATOR_KA = 0.0;

    public static double ELEVATOR_MAX_VEL = 1.0; // u/s
    public static double ELEVATOR_PROFILE_MAX_VEL = 1.0; // u/s
    public static double ELEVATOR_PROFILE_MAX_ACC = 1.0; // u/s^2
    public static double ELEVATOR_CONV_FACTOR = 1 / 20.0;

    public static final int ELEVATOR_PID_SLOT_POS = 0;
    public static final int ELEVATOR_PID_SLOT_VEL = 1;
    public static final double[] ELEVATOR_PID = {0, 0, 0};
    public static final double[] ELEVATOR_VEL_PIF = {0.1, 0.0, 0.001};

    // Indexer Constants
    public static int INDEXER_CONVEYOR_TOP_MOTOR_ID = 7;
    public static int INDEXER_CONVEYOR_BOTTOM_MOTOR_ID = 8;
    public static int INDEXER_STOP_MOTOR_ID = 5;
    public static int INDEXER_BOTTOM_BREAKBEAM_ID = 1;

    public static double INDEXER_CONVEYOR_NEUTRAL_SPEED = 0.0;
    public static double INDEXER_CONVEYOR_LOWER_SPEED = 0.5;
    public static double INDEXER_CONVEYOR_RAISE_SPEED = -0.75;
    public static double INDEXER_CONVEYOR_SHOOT_SPEED = -0.5;

    public static double INDEXER_STOP_NEUTRAL_SPEED = 0.0;
    public static double INDEXER_STOP_SHOOT_SPEED = -1.0;

    public static double[] INDEXER_PID = {0, 0, 0};
    public static double INDEXER_ADVANCE_SETPOINT = 10.0;
    public static double INDEXER_PID_TOLERANCE = 0.5;

    public static int INDEXER_TOP_SENSOR_NUM_AVG = 5;
    public static double INDEXER_TOP_BALL_PROXIMITY = 300;

    // Intake Constants
    public static int INTAKE_MOTOR_ID = 9;
    public static int INTAKE_SERVO_ID = 0;

    public static double INTAKE_SERVO_RELEASE_SETPOINT = 1.0;
    public static double INTAKE_NEUTRAL_SPEED = 0.0;
    public static double INTAKE_INTAKE_SPEED = -0.8;
    public static double INTAKE_EJECT_SPEED = 0.8;

    // Shooter Constants
    public static int SHOOTER_MOTOR_ID = 11;
    public static int SHOOTER_FOLLOWER_MOTOR_ID = 12;

    public static double SHOOTER_NEUTRAL_SPEED = 0;
    public static double SHOOTER_OPEN_LOOP_SPEED = 0.9;

    public static double[] SHOOTER_PIDF = {0.000001, 0.0, 0.0, 0.000180};
    public static double SHOOTER_VEL_SETPOINT = 4500;
    public static double SHOOTER_PERCENT_TOLERANCE = 3; // in %

    // Drivetrain Constants
    public static int DRIVE_FRONT_LEFT = 1;
    public static int DRIVE_FRONT_RIGHT = 2;
    public static int DRIVE_BACK_LEFT = 3;
    public static int DRIVE_BACK_RIGHT = 4;

    public static double DRIVE_TRACK_WIDTH_M = 1.0; // m
    public static double DRIVE_WHEEL_DIAM_M = 0.1524; // m
    public static double DRIVE_GEARBOX_REDUCTION = 10.71;

    public static double DRIVE_REDUCE_TURNING_CONSTANT = 0.8;

    public static int DRIVE_PID_SLOT_VEL = 0;
    public static double DRIVE_LEFT_VEL_PID_P = 0.1;
    public static double DRIVE_LEFT_VEL_PID_I = 0.0;
    public static double DRIVE_RIGHT_VEL_PID_P = 0.1;
    public static double DRIVE_RIGHT_VEL_PID_I = 0.0;
    public static double DRIVE_MAX_VEL = 3.0; // m/s
    public static double DRIVE_MAX_ROT = 1.0; // rad/s

    public static double[] DRIVE_DIST_PID = {0.1, 0.0, 0.0};
    public static double DRIVE_DIST_PID_TOLERANCE = 0.1;
    public static double DRIVE_MAINTAIN_ANGLE_PID_P = 0.1;

    public static double[] DRIVE_ANGLE_PID = {0.1, 0.0, 0.0};
    public static double DRIVE_ANGLE_PID_TOLERANCE = 0.1;

    public static double DRIVE_KS = 0.144;
    public static double DRIVE_KV = 2.83;
    public static double DRIVE_KA = 0.47;

    public static double DRIVE_PROFILE_MAX_VEL = 3.0; // m/s
    public static double DRIVE_PROFILE_MAX_ACC = 5.0; // m/s^2
    public static double DRIVE_PROFILE_RIGHT_POS_P = 0.1;
    public static double DRIVE_PROFILE_LEFT_POS_P = 0.1;

    public static double DRIVE_RAMSETE_B = 2.0;
    public static double DRIVE_RAMSETE_ZETA = 0.7;

    public static double PI = 3.14159265;
}
