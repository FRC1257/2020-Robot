package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

/**
 * Subsystem to handle the intake mechanism
 * 
 * - Utilizes one NEO 550 motor attached to the intake mechanism
 */

public class Intake extends SnailSubsystem {

    private final CANSparkMax intakeMotor;

    private final Servo intakeReleaseServo;
    private boolean isReleased;

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
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        intakeReleaseServo = new Servo(INTAKE_SERVO_ID);
        isReleased = false;
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
        if (isReleased) {
            intakeReleaseServo.set(INTAKE_SERVO_RELEASE_SETPOINT);
        }
        else {
            intakeReleaseServo.set(0);
        }
    }
    
    /**
     * Puts relevant values to Smart Dashboard
     */
    @Override
    public void outputValues() {
        SmartDashboard.putString("Intake State", state.name());
        SmartDashboard.putNumber("Intake Motor Current", intakeMotor.getOutputCurrent());
    }

    /**
     * Puts values that can be changed into Smart Dashboard
     */
    @Override
    public void setConstantTuning() {
        SmartDashboard.putNumber("Intake Eject Speed", INTAKE_EJECT_SPEED);
        SmartDashboard.putNumber("Intake Intake Speed", INTAKE_INTAKE_SPEED);
        SmartDashboard.putNumber("Intake Neutral Speed", INTAKE_NEUTRAL_SPEED);
    }

    /**
     * Gets values that can be changed
     */
    @Override
    public void getConstantTuning() {
        INTAKE_EJECT_SPEED = SmartDashboard.getNumber("Intake Eject Speed", INTAKE_EJECT_SPEED);
        INTAKE_INTAKE_SPEED = SmartDashboard.getNumber("Intake Intake Speed", INTAKE_INTAKE_SPEED);
        INTAKE_NEUTRAL_SPEED = SmartDashboard.getNumber("Intake Neutral Speed", INTAKE_NEUTRAL_SPEED);
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
    * Toggle the servo to release the intake
    */
    public void toggleReleaseIntake() {
        isReleased = (!isReleased);
    }

    /**
    * Returns the state
    */
    public State getState() {
        return state;
    }
}
