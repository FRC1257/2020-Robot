package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem to handle the intake mechanism
 * 
 * - Utilizes one NEO 550 motor attached to the intake mechanism
 */

public class Intake extends SubsystemBase {

    CANSparkMax intakeMotor;

    /**
     * NEUTRAL - The power cells are not moved by the intake
     * 
     * INTAKING - The power cells are intaked and given to the indexer
     * 
     * EJECTING - THe power cells are ejected from the intake and taken out of the robot's control
     */
    public enum State {
        NEUTRAL,
        INTAKING,
        EJECTING
    }
    
    State state = State.NEUTRAL;

    public Intake() {
       intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
       intakeMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);

       setConstantTuning();
    }
    
    /**
     * Update motor outputs according to the current state
     */
    @Override
    public void periodic() {
        switch(state) {
            case NEUTRAL: 
                intakeMotor.set(INTAKE_NEUTRAL_SPEED);
                break;
            case INTAKING:
                intakeMotor.set(INTAKE_INTAKE_SPEED);
                break;
            case EJECTING:
                intakeMotor.set(INTAKE_EJECT_SPEED);
                break;
        }
    }
    
    /**
     * Puts relevant values to Smart Dashboard
     */
    public void outputValues() {
        SmartDashboard.putString("Intake State", state.name());
        SmartDashboard.putNumber("Intake Motor Current", intakeMotor.getOutputCurrent());
    }

    /**
     * Puts values that can be changed into Smart Dashboard
     */
    private void setConstantTuning() {
        SmartDashboard.putNumber("Intake Intake Speed", INDEXER_CONVEYOR_INTAKE_SPEED);
        SmartDashboard.putNumber("Intake Shoot Speed", INDEXER_CONVEYOR_SHOOT_SPEED);
        SmartDashboard.putNumber("Intake Eject Speed", INDEXER_CONVEYOR_EJECT_SPEED);
    }

    /**
     * Gets values that can be changed
     */
    public void getConstantTuning() {
        if (INTAKE_EJECT_SPEED != SmartDashboard.getNumber("Intake Eject Speed", INTAKE_EJECT_SPEED)) {
            INTAKE_EJECT_SPEED = SmartDashboard.getNumber("Intake Eject Speed", INTAKE_EJECT_SPEED);
        }
        if (INTAKE_INTAKE_SPEED != SmartDashboard.getNumber("Intake Intake Speed", INTAKE_INTAKE_SPEED)) {
            INTAKE_INTAKE_SPEED = SmartDashboard.getNumber("Intake Intake Speed", INTAKE_INTAKE_SPEED);
        }
        if (INTAKE_NEUTRAL_SPEED != SmartDashboard.getNumber("Intake Neutral Speed", INTAKE_NEUTRAL_SPEED)) {
            INTAKE_NEUTRAL_SPEED = SmartDashboard.getNumber("Intake Neutral Speed", INTAKE_NEUTRAL_SPEED);
        }
    }

    /**
    * Changes state to neutral
    */
    public void neutral() {
        state = State.NEUTRAL;
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
    public State getState() {
        return state;
    }
}
