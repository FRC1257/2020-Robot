package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

        PIDconveyerTop.setP(INDEXER_PIDF[0]);
        PIDconveyerTop.setI(INDEXER_PIDF[1]);
        PIDconveyerTop.setD(INDEXER_PIDF[2]);
        PIDconveyerTop.setFF(INDEXER_PIDF[3]);

        stopMotor = new CANSparkMax(INDEXER_STOP_MOTOR_ID, MotorType.kBrushless);
        stopMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMITER);

        setConstantTuning();
    }
    
    @Override
    public void periodic() {
        switch(state) {
            case NEUTRAL: 
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
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);                  
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

    public void outputValues() {
        SmartDashboard.putString("Cargo Arm State", state.name());

        SmartDashboard.putNumber("Cargo Arm PID Setpoint", PIDSetpoint);
        SmartDashboard.putNumber("Cargo Arm Position", getEncoderVal());

        SmartDashboard.putNumber("conveyerMotorTop Current", conveyerMotorTop.getOutputCurrent());
        SmartDashboard.putNumber("conveyerMotorTop Temperature (C)", conveyerMotorTop.getMotorTemperature());

        SmartDashboard.putNumber("conveyerMotorBottom Current", conveyerMotorBottom.getOutputCurrent());
        SmartDashboard.putNumber("conveyerMotorBottom Temperature (C)", conveyerMotorBottom.getMotorTemperature());

        SmartDashboard.putNumber("stopMotor Current", stopMotor.getOutputCurrent());
        SmartDashboard.putNumber("stopMotor Current Temperature (C)", stopMotor.getMotorTemperature());
    }
    private void setConstantTuning() {
        SmartDashboard.putNumber("Cargo Arm Max Speed", INDEXER_MAX_SPEED);

        SmartDashboard.putNumber("Cargo Arm P", INDEXER_PIDF[0]);
        SmartDashboard.putNumber("Cargo Arm I", INDEXER_PIDF[1]);
        SmartDashboard.putNumber("Cargo Arm D", INDEXER_PIDF[2]);
        SmartDashboard.putNumber("Cargo Arm F", INDEXER_PIDF[3]);
    }

    public void getConstantTuning() {
        INDEXER_MAX_SPEED = SmartDashboard.getNumber("Cargo Arm Max Speed", INDEXER_MAX_SPEED);

        if (INDEXER_PIDF[0] != SmartDashboard.getNumber("Cargo Arm P", INDEXER_PIDF[0])) {
            INDEXER_PIDF[0] = SmartDashboard.getNumber("Cargo Arm P", INDEXER_PIDF[0]);
            PIDconveyerTop.setP(INDEXER_PIDF[0]);
        }
        if (INDEXER_PIDF[1] != SmartDashboard.getNumber("Cargo Arm I", INDEXER_PIDF[1])) {
            INDEXER_PIDF[1] = SmartDashboard.getNumber("Cargo Arm I", INDEXER_PIDF[1]);
            PIDconveyerTop.setP(INDEXER_PIDF[1]);
        }
        if (INDEXER_PIDF[2] != SmartDashboard.getNumber("Cargo Arm D", INDEXER_PIDF[2])) {
            INDEXER_PIDF[2] = SmartDashboard.getNumber("Cargo Arm D", INDEXER_PIDF[2]);
            PIDconveyerTop.setP(INDEXER_PIDF[2]);
        }
        if (INDEXER_PIDF[3] != SmartDashboard.getNumber("Cargo Arm F", INDEXER_PIDF[3])) {
            INDEXER_PIDF[3] = SmartDashboard.getNumber("Cargo Arm F", INDEXER_PIDF[3]);
            PIDconveyerTop.setP(INDEXER_PIDF[3]);
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
        resetEncoder();
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
    public State getState(){
        return state;
    }

}
