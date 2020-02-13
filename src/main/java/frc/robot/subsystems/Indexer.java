package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem to handle the conveyer belt and the stop mechanism
 * 
 * - Utilizes two NEO 550 motors attached to the conveyer
 * 
 * - Utilizes one NEO 550 motor attached to the stop mechanism
 * 
 * - Uses PID to move the Indexer to specific setpoints
 */

public class Indexer extends SnailSubsystem {

    private CANSparkMax conveyorMotorTop;
    private CANEncoder conveyorEncoder;
    private CANPIDController conveyorPID;

    private CANSparkMax conveyorMotorBottom;

    private CANSparkMax stopMotor;

    private double currentPIDSetpoint;

    /**
     * NEUTRAL - The position of each of the power cells is maintained
     * 
     * PID - Using PID control to go to and maintain a specific position
     * 
     * SHOOTING - The power cells are put into the shooter
     * 
     * INTAKING - The power cells are intaked higher into the indexer
     * 
     * EJECTING - THe power cells are ejected from the indexer to the intake
     */
    public enum State {
        NEUTRAL,
        PID,
        SHOOTING,
        INTAKING,
        EJECTING
    }
    State state = State.NEUTRAL;

    public Indexer() {
        conveyorMotorTop = new CANSparkMax(INDEXER_CONVEYOR_TOP_MOTOR_ID, MotorType.kBrushless);
        conveyorMotorTop.restoreFactoryDefaults();
        conveyorMotorTop.setIdleMode(IdleMode.kBrake);
        conveyorMotorTop.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        conveyorEncoder = conveyorMotorTop.getEncoder();
        conveyorPID = conveyorMotorTop.getPIDController();

        conveyorMotorBottom = new CANSparkMax(INDEXER_CONVEYOR_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        conveyorMotorBottom.restoreFactoryDefaults();
        conveyorMotorBottom.setIdleMode(IdleMode.kBrake);
        conveyorMotorBottom.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        conveyorMotorBottom.follow(conveyorMotorTop);

        conveyorPID.setP(INDEXER_PID[0]);
        conveyorPID.setI(INDEXER_PID[1]);
        conveyorPID.setD(INDEXER_PID[2]);
        conveyorPID.setFF(INDEXER_PID[3]);

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
                conveyorMotorTop.set(INDEXER_CONVEYOR_NEUTRAL_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);                
                break;
            case SHOOTING:
                conveyorMotorTop.set(INDEXER_CONVEYOR_SHOOT_SPEED);
                stopMotor.set(INDEXER_STOP_SHOOT_SPEED);
                break;
            case INTAKING:
                conveyorMotorTop.set(INDEXER_CONVEYOR_INTAKE_SPEED);
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
            case EJECTING:
                conveyorMotorTop.set(INDEXER_CONVEYOR_EJECT_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
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

        SmartDashboard.putNumber("Conveyor Top Current", conveyorMotorTop.getOutputCurrent());
        SmartDashboard.putNumber("Top Encoder", getEncoderValue());

        SmartDashboard.putNumber("Conveyor Bottom Current", conveyorMotorBottom.getOutputCurrent());
        SmartDashboard.putNumber("Bottom Encoder", getEncoderValue());

        SmartDashboard.putNumber("Stop Motor Current", stopMotor.getOutputCurrent());
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

        SmartDashboard.putNumber("Indexer Conveyer Intake Speed", INDEXER_CONVEYOR_INTAKE_SPEED);
        SmartDashboard.putNumber("Indexer Conveyer Shoot Speed", INDEXER_CONVEYOR_SHOOT_SPEED);
        SmartDashboard.putNumber("Indexer Conveyer Eject Speed", INDEXER_CONVEYOR_EJECT_SPEED);
        SmartDashboard.putNumber("Indexer Conveyer Neutral Speed", INDEXER_CONVEYOR_NEUTRAL_SPEED);
    }

    /**
     * Gets values that can be changed
     */
    @Override
    public void getConstantTuning() {
        if (conveyorPID.getP() != SmartDashboard.getNumber("Indexer PID kP", INDEXER_PID[0])) {
            INDEXER_PID[0] = SmartDashboard.getNumber("Indexer PID kP", INDEXER_PID[0]);
            conveyorPID.setP(INDEXER_PID[0]);
        }
        if (conveyorPID.getI() != SmartDashboard.getNumber("Indexer PID kI", INDEXER_PID[1])) {
            INDEXER_PID[1] = SmartDashboard.getNumber("Indexer PID kI", INDEXER_PID[1]);
            conveyorPID.setP(INDEXER_PID[1]);
        }
        if (conveyorPID.getD() != SmartDashboard.getNumber("Indexer PID kD", INDEXER_PID[2])) {
            INDEXER_PID[2] = SmartDashboard.getNumber( "Indexer PID kD", INDEXER_PID[2]);
            conveyorPID.setP(INDEXER_PID[2]);
        }
        
        INDEXER_ADVANCE_SETPOINT = SmartDashboard.getNumber("Indexer Advance Setpoint", INDEXER_ADVANCE_SETPOINT);

        INDEXER_STOP_SHOOT_SPEED = SmartDashboard.getNumber("Indexer Stop Shoot Speed", INDEXER_STOP_SHOOT_SPEED);
        INDEXER_STOP_NEUTRAL_SPEED = SmartDashboard.getNumber("Indexer Stop Neutral Speed", INDEXER_STOP_NEUTRAL_SPEED);

        INDEXER_CONVEYOR_SHOOT_SPEED = SmartDashboard.getNumber("Indexer Conveyer Shoot Speed", INDEXER_CONVEYOR_SHOOT_SPEED);
        INDEXER_CONVEYOR_EJECT_SPEED = SmartDashboard.getNumber("Indexer Conveyer Eject Speed", INDEXER_CONVEYOR_EJECT_SPEED);
        INDEXER_CONVEYOR_INTAKE_SPEED = SmartDashboard.getNumber("Indexer Conveyer Intake Speed", INDEXER_CONVEYOR_INTAKE_SPEED);
        INDEXER_CONVEYOR_NEUTRAL_SPEED = SmartDashboard.getNumber("Indexer Conveyer Neutral Speed", INDEXER_CONVEYOR_NEUTRAL_SPEED);
    }
     
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
    * Move the the conveyer to the next position
    */
    public void advance() {
        setCurrentPIDSetpoint(INDEXER_ADVANCE_SETPOINT);
    }
    
    /**
    * Changes state to neutral
    */
    public void neutral() {
        state = State.NEUTRAL;
    }

    /**
    * Changes state to shoot
    */
    public void shoot() {
        state = State.SHOOTING;
    }

    /**
    * Changes state to eject
    */
    public void eject() {
        state = State.EJECTING;
    }

    /**
    * Changes state to intake
    */
    public void intake() {
        state = State.INTAKING;
    }

    /**
    * returns the state
    */
    public State getState(){
        return state;
    }
}
