package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SnailSubsystem {

    CANSparkMax intakeMotor;

    Servo intakeReleaseServo;
    boolean isReleased;

    public enum State {
        NEUTRAL,
        INTAKING,
        EJECTING
    }
    
    State state = State.NEUTRAL;

    public Intake() {
        intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        intakeReleaseServo = new Servo(INTAKE_SERVO_ID);
        isReleased = false;
    }
    
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
        if(isReleased){
            intakeReleaseServo.set(INTAKE_SERVO_RELEASE_SETPOINT);
        }
    }
    
    public void outputValues() {
        SmartDashboard.putString("Intake State", state.name());
        SmartDashboard.putNumber("Intake Motor Current", intakeMotor.getOutputCurrent());
    }

    public void setConstantTuning() {
        SmartDashboard.putNumber("Intake Intake Speed", INDEXER_CONVEYOR_INTAKE_SPEED);
        SmartDashboard.putNumber("Intake Shoot Speed", INDEXER_CONVEYOR_SHOOT_SPEED);
        SmartDashboard.putNumber("Intake Eject Speed", INDEXER_CONVEYOR_EJECT_SPEED);
    }

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

    public void neutral() {
        state = State.NEUTRAL;
    }

    public void eject() {
        state = State.EJECTING;
    }

    public void intake() {
        state = State.INTAKING;
    }

    public void toggleReleaseIntake() {
        isReleased = (!isReleased);
    }

    public State getState() {
        return state;
    }
}
