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

    CANSparkMax conveyerMotorTop;
    CANEncoder conveyerEncoderTop;
    CANPIDController PIDconveyerTop;

    CANSparkMax conveyerMotorBottom;
    CANEncoder conveyerEncoderBottom;
    CANPIDController PIDconveyerBottom;

    CANSparkMax stopMotor;

    private double PIDSetpoint;


    public enum State {
        NEUTRAL,
        SHOOTING,
        INTAKING,
        EJECTING,
        PID
    }
    State state = State.NEUTRAL;

    public Indexer() {
        conveyerMotorTop = new CANSparkMax(INDEXER_CONVEYER_TOP_MOTOR_ID, MotorType.kBrushless);
        conveyerMotorTop.restoreFactoryDefaults();
        conveyerMotorTop.setIdleMode(IdleMode.kBrake);
        conveyerMotorTop.setSmartCurrentLimit(NEO_550_CURRENT_LIMITER);
        conveyerEncoderTop = conveyerMotorTop.getEncoder();
        PIDconveyerTop = conveyerMotorTop.getPIDController();

        conveyerMotorBottom = new CANSparkMax(INDEXER_CONVEYER_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        conveyerMotorBottom.restoreFactoryDefaults();
        conveyerMotorBottom.setIdleMode(IdleMode.kBrake);
        conveyerMotorBottom.setSmartCurrentLimit(NEO_550_CURRENT_LIMITER);

        conveyerMotorBottom.follow(conveyerMotorTop);

        PIDconveyerTop.setP(PID_CONVEYER_CONSTS[0]);
        PIDconveyerTop.setI(PID_CONVEYER_CONSTS[1]);
        PIDconveyerTop.setD(PID_CONVEYER_CONSTS[2]);
        PIDconveyerTop.setFF(PID_CONVEYER_CONSTS[3]);

        stopMotor = new CANSparkMax(INDEXER_STOP_MOTOR_ID, MotorType.kBrushless);
        stopMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMITER);
    }
    
    @Override
    public void periodic() {
        switch(state) {
            case NEUTRAL: 
                resetEncoder();
                PIDSetpoint = 0;
                conveyerMotorTop.set(INDEXER_CONVEYER_NEUTRAL_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);                
                break;
            case SHOOTING:
                PIDSetpoint = 0;
                conveyerMotorTop.set(INDEXER_CONVEYER_SHOOT_SPEED);
                stopMotor.set(INDEXER_STOP_SHOOT_SPEED);
                break;
            case INTAKING:
                PIDSetpoint = 0;
                conveyerMotorTop.set(INDEXER_CONVEYER_INTAKE_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
                break;
            case PID:            
                if (PIDSetpoint == 0) {
                    state = State.NEUTRAL;
                } 
                else {
                    PIDconveyerTop.setReference(PIDSetpoint,ControlType.kPosition);
                }
                break;
            case EJECTING:
                conveyerMotorTop.set(INDEXER_CONVEYER_EJECT_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
                break;
        }
    }

     
    public double getEncoderVal() {
        return conveyerEncoderTop.getPosition();
    }

    public void resetEncoder() {
        conveyerEncoderTop.setPosition(0.0);
    }
    public double getPIDSetpoint( ){
        return PIDSetpoint;
    }
    public void setPIDSetpoint(double value) {
        PIDSetpoint = value;
        state = State.PID;
    }

    public void moveOneIndex() {
        setPIDSetpoint(ONE_INDEX_SETPOINT);
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
    public void pid() {
        state = State.PID;
    }
}
