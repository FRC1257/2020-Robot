package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    CANSparkMax conveyerMotor;
    CANEncoder conveyerEncoder;
    CANPIDController PIDconveyer;

    CANSparkMax stopMotor;


    public enum State {
        NEUTRAL,
        SHOOTING,
        INTAKING,
        EJECTING
    }
    State state = State.NEUTRAL;

    public Indexer() {
        conveyerMotor = new CANSparkMax(INDEXER_CONVEYER_MOTOR_ID, MotorType.kBrushless);
        conveyerMotor.restoreFactoryDefaults();
        conveyerMotor.setIdleMode(IdleMode.kBrake);
        conveyerMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMITER);
        conveyerEncoder = conveyerMotor.getEncoder();
        PIDconveyer = conveyerMotor.getPIDController();

        stopMotor = new CANSparkMax(INDEXER_STOP_MOTOR_ID, MotorType.kBrushless);
        stopMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMITER);
    }
    
    @Override
    public void periodic() {
        switch(state) {
            case NEUTRAL: 
                conveyerMotor.set(INDEXER_CONVEYER_NEUTRAL_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);                
                break;
            case SHOOTING:
                conveyerMotor.set(INDEXER_CONVEYER_SHOOT_SPEED);
                stopMotor.set(INDEXER_STOP_SHOOT_SPEED);
                break;
            case INTAKING:
                conveyerMotor.set(INDEXER_CONVEYER_INTAKE_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
                PIDconveyer.setReference(INDEXER_PID_SETPOINT,ControlType.kPosition);
                break;
            case EJECTING:
                conveyerMotor.set(INDEXER_CONVEYER_EJECT_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
                break;
        }
    }

    public double getEncoderVal(){
        return conveyerEncoder.getPosition();
    }

    public void neutral() {
        state = State.NEUTRAL;
    }

    public void shoot() {
        state = State.SHOOTING;
    }

    public void eject() {
        state = State.EJECTING;
    }

    public void intake() {
        state = State.INTAKING;
    }
}
