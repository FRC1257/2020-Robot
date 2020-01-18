package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    CANSparkMax conveyerMotor;
    CANSparkMax stopMotor;


    public enum State {
        NEUTRAL,
        SHOOT,
        INTAKE,
        EJECT
    }
    State state = State.NEUTRAL;

    public Indexer() {
        conveyerMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        stopMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    }
    
    @Override
    public void periodic() {
        switch(state) {
            case NEUTRAL: 
                conveyerMotor.set(INDEXER_CONVEYER_NEUTRAL_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);                
                break;
            case SHOOT:
                conveyerMotor.set(INDEXER_CONVEYER_SHOOT_SPEED);
                stopMotor.set(INDEXER_STOP_SHOOT_SPEED);
                break;
            case INTAKE:
                conveyerMotor.set(INDEXER_CONVEYER_INTAKE_SPEED);
                stopMotor.set(INDEXER_STOP_SHOOT_SPEED);
                break;
            case EJECT:
                conveyerMotor.set(INDEXER_CONVEYER_EJECT_SPEED);
                stopMotor.set(INDEXER_STOP_SHOOT_SPEED);
                break;
        }
    }
    public void neutral() {
        state = State.NEUTRAL;
    }

    public void shoot() {
        state = State.SHOOT;
    }

    public void eject() {
        state = State.EJECT;
    }
    public void intake(){
        state = State.INTAKE;
    }
}