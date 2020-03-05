package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

/**
 * Subsystem to handle the conveyor belt and the stop mechanism
 * 
 * - Utilizes two NEO 550 motors attached to the conveyor
 * 
 * - Utilizes one NEO 550 motor attached to the stop mechanism
 * 
 * - Uses PID to move the Indexer to specific setpoints
 */

public class Indexer extends SnailSubsystem {

    private final CANSparkMax conveyorMotorFront;
    private final CANSparkMax conveyorMotorBack;
    
    private final CANEncoder conveyorEncoder;
    private final CANPIDController conveyorPID;

    private final CANSparkMax stopMotor;

    private double currentPIDSetpoint;

    private DigitalInput bottomFrontBreakbeam;
    private DigitalInput bottomBackBreakbeam;
    private ColorSensorV3 colorSensor;
    private MedianFilter filter;
    private double lastFilteredDist;
    private boolean override;

    private Notifier looper;

    /**
     * NEUTRAL - The position of each of the power cells is maintained
     * 
     * PID - Using PID control to go to and maintain a specific position
     * 
     * SHOOTING - The power cells are put into the shooter
     * 
     * RAISING - The power cells are raised higher into the indexer
     * 
     * LOWERING - THe power cells are lowered from the indexer to the intake
     */
    
    public enum State {
        NEUTRAL,
        PID,
        SHOOTING, // manual
        RAISING,  // manual
        LOWERING, // manual
        CELL_RAISING,   // cell was just detected and the indexer should now be rising up
        CELL_RETURNING, // cell has just left breakbeam and is being put back in
        CELL_NUDGING    // cell has reentered breakbeam and is being nudged up
    }
    State state = State.NEUTRAL;

    public Indexer() {
        conveyorMotorFront = new CANSparkMax(INDEXER_CONVEYOR_TOP_MOTOR_ID, MotorType.kBrushless);
        conveyorMotorFront.restoreFactoryDefaults();
        conveyorMotorFront.setIdleMode(IdleMode.kBrake);
        conveyorMotorFront.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        conveyorMotorFront.setInverted(true);
        conveyorEncoder = conveyorMotorFront.getEncoder();
        conveyorPID = conveyorMotorFront.getPIDController();

        conveyorMotorBack = new CANSparkMax(INDEXER_CONVEYOR_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        conveyorMotorBack.restoreFactoryDefaults();
        conveyorMotorBack.setIdleMode(IdleMode.kBrake);
        conveyorMotorBack.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        conveyorMotorBack.follow(conveyorMotorFront, false);

        conveyorPID.setP(INDEXER_PID[0]);
        conveyorPID.setI(INDEXER_PID[1]);
        conveyorPID.setD(INDEXER_PID[2]);

        stopMotor = new CANSparkMax(INDEXER_STOP_MOTOR_ID, MotorType.kBrushless);
        stopMotor.restoreFactoryDefaults();
        stopMotor.setIdleMode(IdleMode.kBrake);
        stopMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);

        bottomFrontBreakbeam = new DigitalInput(INDEXER_BOTTOM_BREAKBEAM_FRONT_ID);
        bottomBackBreakbeam = new DigitalInput(INDEXER_BOTTOM_BREAKBEAM_BACK_ID);
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        filter = new MedianFilter(INDEXER_TOP_SENSOR_NUM_MED);

        looper = new Notifier(this::updateNotifier);
        looper.startPeriodic(INDEXER_LOOPER_PERIOD);

        reset();
    }

    private void reset() {
        currentPIDSetpoint = -1257.0;
        lastFilteredDist = 0;
    }

    /**
     * Update motor outputs according to the current state
     * This method is called within a notifier to achieve a much faster refresh rate and in a separate thread
     */
    private void updateNotifier() {
        updateDistance();

        switch(state) {
            case NEUTRAL:
                conveyorMotorFront.set(INDEXER_CONVEYOR_NEUTRAL_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
                break;
            case SHOOTING:
                conveyorMotorFront.set(INDEXER_CONVEYOR_SHOOT_SPEED);
                stopMotor.set(INDEXER_STOP_SHOOT_SPEED);
                break;
            case RAISING:
                if (canMove() || override) {
                    conveyorMotorFront.set(INDEXER_CONVEYOR_RAISE_SPEED);
                }
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
                break;
            case LOWERING:
                conveyorMotorFront.set(INDEXER_CONVEYOR_LOWER_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
                break;
            case PID:
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
                if (currentPIDSetpoint == -1257.0) {
                    state = State.NEUTRAL;
                    break;
                }

                conveyorPID.setReference(currentPIDSetpoint, ControlType.kPosition);

                if (Math.abs(conveyorEncoder.getPosition() - currentPIDSetpoint) < INDEXER_PID_TOLERANCE) {
                    state = State.NEUTRAL;
                }
                break;

            // automatic indexing
            case CELL_RAISING:
                if(!ballAtBot()) {
                    state = State.CELL_RETURNING;
                }
                else {
                    conveyorMotorFront.set(INDEXER_CONVEYOR_RAISE_SPEED);
                    stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
                }
                break;
            case CELL_RETURNING:
                if(ballAtBot()) {
                    state = State.CELL_NUDGING;
                }
                else {
                    conveyorMotorFront.set(INDEXER_CONVEYOR_RETURN_SPEED);
                    stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
                }
                break;
            case CELL_NUDGING:
                if(!ballAtBot()) {
                    state = State.NEUTRAL;
                }
                else {
                    conveyorMotorFront.set(INDEXER_CONVEYOR_NUDGE_SPEED);
                }
                break;
        }
    }

    @Override
    public void periodic() {

    }

    /**
     * Puts relevant values to Smart Dashboard
     */
    @Override
    public void outputValues() {
        SmartDashboard.putString("Indexer State", state.name());

        SmartDashboard.putNumber("Indexer PID Setpoint", currentPIDSetpoint);
        SmartDashboard.putNumber("Indexer Encoder", getEncoderValue());

        SmartDashboard.putNumber("Indexer Color Sensor", lastFilteredDist);
        SmartDashboard.putBoolean("Indexer Breakbeam Front", bottomFrontBreakbeam.get());
        SmartDashboard.putBoolean("Indexer Breakbeam Back", bottomBackBreakbeam.get());
        SmartDashboard.putBoolean("Indexer Can Move", canMove());

        SmartDashboard.putNumber("Indexer Front Conveyor Current", conveyorMotorFront.getOutputCurrent());
        SmartDashboard.putNumber("Indexer Back Conveyor Current", conveyorMotorBack.getOutputCurrent());
        SmartDashboard.putNumber("Indexer Stop Motor Current", stopMotor.getOutputCurrent());
    }

    /**
     * Puts values that can be changed into Smart Dashboard
     */
    @Override
    public void setConstantTuning() {
        SmartDashboard.putNumber("Indexer PID kP", INDEXER_PID[0]);
        SmartDashboard.putNumber("Indexer PID kI", INDEXER_PID[1]);
        SmartDashboard.putNumber("Indexer PID kD", INDEXER_PID[2]);

        SmartDashboard.putNumber("Indexer Advance Setpoint", INDEXER_ADVANCE_SETPOINT);

        SmartDashboard.putNumber("Indexer Stop Shoot Speed", INDEXER_STOP_SHOOT_SPEED);
        SmartDashboard.putNumber("Indexer Stop Neutral Speed", INDEXER_STOP_NEUTRAL_SPEED);

        SmartDashboard.putNumber("Indexer Conveyor Raise Speed", INDEXER_CONVEYOR_RAISE_SPEED);
        SmartDashboard.putNumber("Indexer Conveyor Lower Speed", INDEXER_CONVEYOR_LOWER_SPEED);
        SmartDashboard.putNumber("Indexer Conveyor Shoot Speed", INDEXER_CONVEYOR_SHOOT_SPEED);
        SmartDashboard.putNumber("Indexer Conveyor Neutral Speed", INDEXER_CONVEYOR_NEUTRAL_SPEED);
    }

    /**
     * Gets values that can be changed
     */
    @Override
    public void getConstantTuning() {
        INDEXER_PID[0] = SmartDashboard.getNumber("Indexer PID kP", INDEXER_PID[0]);
        INDEXER_PID[1] = SmartDashboard.getNumber("Indexer PID kI", INDEXER_PID[1]);
        INDEXER_PID[2] = SmartDashboard.getNumber("Indexer PID kD", INDEXER_PID[2]);

        INDEXER_ADVANCE_SETPOINT = SmartDashboard.getNumber("Indexer Advance Setpoint", INDEXER_ADVANCE_SETPOINT);

        INDEXER_STOP_SHOOT_SPEED = SmartDashboard.getNumber("Indexer Stop Shoot Speed", INDEXER_STOP_SHOOT_SPEED);
        INDEXER_STOP_NEUTRAL_SPEED = SmartDashboard.getNumber("Indexer Stop Neutral Speed", INDEXER_STOP_NEUTRAL_SPEED);

        INDEXER_CONVEYOR_SHOOT_SPEED = SmartDashboard.getNumber("Indexer Conveyor Shoot Speed", INDEXER_CONVEYOR_SHOOT_SPEED);
        INDEXER_CONVEYOR_RAISE_SPEED = SmartDashboard.getNumber("Indexer Conveyor Raise Speed", INDEXER_CONVEYOR_RAISE_SPEED);
        INDEXER_CONVEYOR_LOWER_SPEED = SmartDashboard.getNumber("Indexer Conveyor Lower Speed", INDEXER_CONVEYOR_LOWER_SPEED);
        INDEXER_CONVEYOR_NEUTRAL_SPEED = SmartDashboard.getNumber("Indexer Conveyor Neutral Speed", INDEXER_CONVEYOR_NEUTRAL_SPEED);

        if (conveyorPID.getP() != INDEXER_PID[0]) {
            conveyorPID.setP(INDEXER_PID[0]);
        }
        if (conveyorPID.getP() != INDEXER_PID[0]) {
            conveyorPID.setP(INDEXER_PID[0]);
        }
        if (conveyorPID.getP() != INDEXER_PID[0]) {
            conveyorPID.setP(INDEXER_PID[0]);
        }}

    /**
     * Gets the position of the encoder
     */
    public double getEncoderValue() {
        return conveyorEncoder.getPosition();
    }

    /**
     * Reset the Encoder position to the original position
     */
    public void resetEncoder() {
        conveyorEncoder.setPosition(0);
    }

     /**
     * Move the indexer to a specific setpoint using PID control
     */
    public void setCurrentPIDSetpoint(double value) {
        resetEncoder();
        currentPIDSetpoint = value;
        state = State.PID;
    }
    
    /**
    * Changes state to neutral if not automatically indexing at the moment
    */
    public void neutral() {
        if(state != State.CELL_RAISING && state != State.CELL_RETURNING && state != State.CELL_NUDGING) {
            state = State.NEUTRAL;
        }
    }

    /**
    * Changes state to shooting
    */
    public void shoot() {
        state = State.SHOOTING;
    }

    /**
    * Changes state to lowering
    */
    public void lower() {
        state = State.LOWERING;
    }

    /**
    * Changes state to raising
    */
    public void raise() {
        state = State.RAISING;
    }

    /**
     * Move the the conveyor to the next position
     */
    public void advance() {
        setCurrentPIDSetpoint(INDEXER_ADVANCE_SETPOINT);
    }

    /**
     * Updates the filter for the color sensor distance
     */
    private void updateDistance() {
        lastFilteredDist = filter.calculate(colorSensor.getProximity());
    }

    /**
     * Returns whether or not the color sensor detects a ball at the top
     */
    public boolean ballAtTop() {
        return lastFilteredDist > INDEXER_TOP_BALL_PROXIMITY;
    }

    /**
     * Returns whether or not the breakbeam sensors detect a ball at the bottom
     */
    public boolean ballAtBot() {
        return !bottomBackBreakbeam.get() || !bottomFrontBreakbeam.get();
    }

    /**
     * Returns whether or not the indexer is free to move
     */
    public boolean canMove() {
        return ballAtBot() && !ballAtTop();
    }

    /**
     * Sets up the override for the sensors
     */
    public void setOverride(boolean override) {
        this.override = override;
    }

    /**
    * Returns the state
    */
    public State getState() {
        return state;
    }
}
