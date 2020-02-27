package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
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
        SHOOTING,
        RAISING,
        LOWERING
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

        currentPIDSetpoint = -1257.0;
    }
    
    /**
     * Update motor outputs according to the current state
     */
    @Override
    public void periodic() {
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
                conveyorMotorFront.set(INDEXER_CONVEYOR_RAISE_SPEED);
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
        }
    }

    /**
     * Puts relevant values to Smart Dashboard
     */
    @Override
    public void outputValues() {
        SmartDashboard.putString("Indexer State", state.name());

        SmartDashboard.putNumber("Indexer PID Setpoint", currentPIDSetpoint);
        SmartDashboard.putNumber("Indexer Encoder", getEncoderValue());

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
    * Changes state to neutral
    */
    public void neutral() {
        state = State.NEUTRAL;
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
    * Returns the state
    */
    public State getState() {
        return state;
    }
}
