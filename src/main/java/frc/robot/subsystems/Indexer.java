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

    CANSparkMax conveyorMotorTop;
    CANEncoder conveyorEncoderTop;
    CANPIDController PIDconveyorTop;

    CANSparkMax conveyorMotorBottom;
    CANEncoder conveyorEncoderBottom;
    CANPIDController PIDconveyorBottom;

    CANSparkMax stopMotor;

    private double currentPIDSetpoint;

    public enum State {
        NEUTRAL,
        SHOOTING,
        INTAKING,
        EJECTING,
        PID
    }
    State state = State.NEUTRAL;

    public Indexer() {
        conveyorMotorTop = new CANSparkMax(INDEXER_CONVEYOR_TOP_MOTOR_ID, MotorType.kBrushless);
        conveyorMotorTop.restoreFactoryDefaults();
        conveyorMotorTop.setIdleMode(IdleMode.kBrake);
        conveyorMotorTop.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        conveyorEncoderTop = conveyorMotorTop.getEncoder();
        PIDconveyorTop = conveyorMotorTop.getPIDController();

        conveyorMotorBottom = new CANSparkMax(INDEXER_CONVEYOR_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        conveyorMotorBottom.restoreFactoryDefaults();
        conveyorMotorBottom.setIdleMode(IdleMode.kBrake);
        conveyorMotorBottom.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        conveyorMotorBottom.follow(conveyorMotorTop);

        PIDconveyorTop.setP(INDEXER_PIDF[0]);
        PIDconveyorTop.setI(INDEXER_PIDF[1]);
        PIDconveyorTop.setD(INDEXER_PIDF[2]);
        PIDconveyorTop.setFF(INDEXER_PIDF[3]);

        stopMotor = new CANSparkMax(INDEXER_STOP_MOTOR_ID, MotorType.kBrushless);
        stopMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);

        setConstantTuning();
        currentPIDSetpoint = 0.0;
    }
    
    @Override
    public void periodic() {
        switch(state) {
            case NEUTRAL: 
                currentPIDSetpoint = 0.0;
                conveyorMotorTop.set(INDEXER_CONVEYOR_NEUTRAL_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);                
                break;
            case SHOOTING:
                currentPIDSetpoint = 0.0;
                conveyorMotorTop.set(INDEXER_CONVEYOR_SHOOT_SPEED);
                stopMotor.set(INDEXER_STOP_SHOOT_SPEED);
                break;
            case INTAKING:
                currentPIDSetpoint = 0.0;
                conveyorMotorTop.set(INDEXER_CONVEYOR_INTAKE_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
                break;
            case PID:
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);                  
                if (currentPIDSetpoint == 0.0) {
                    state = State.NEUTRAL;
                } 
                else {
                    PIDconveyorTop.setReference(currentPIDSetpoint,ControlType.kPosition);
                }
                break;
            case EJECTING:
                conveyorMotorTop.set(INDEXER_CONVEYOR_EJECT_SPEED);
                stopMotor.set(INDEXER_STOP_NEUTRAL_SPEED);
                break;
        }
    }

    public void outputValues() {
        SmartDashboard.putString("Indexer State", state.name());

        SmartDashboard.putNumber("Indexer PID Setpoint", currentPIDSetpoint);

        SmartDashboard.putNumber("conveyorMotorTop Current", conveyorMotorTop.getOutputCurrent());
        SmartDashboard.putNumber("TopEncoder Values", getEncoderVal());


        SmartDashboard.putNumber("conveyorMotorBottom Current", conveyorMotorBottom.getOutputCurrent());
        SmartDashboard.putNumber("BottomEncoder Values", getEncoderVal());

        SmartDashboard.putNumber("stopMotor Current", stopMotor.getOutputCurrent());

    }
    private void setConstantTuning() {
        SmartDashboard.putNumber("Indexer P", INDEXER_PIDF[0]);
        SmartDashboard.putNumber("Indexer I", INDEXER_PIDF[1]);
        SmartDashboard.putNumber("Indexer D", INDEXER_PIDF[2]);
        SmartDashboard.putNumber("Indexer F", INDEXER_PIDF[3]);

        SmartDashboard.putNumber("Indexer One Setpoint", ONE_INDEX_SETPOINT);

        SmartDashboard.putNumber("Indexer Stop Shoot Speed", INDEXER_STOP_SHOOT_SPEED);
        SmartDashboard.putNumber("Indexer Stop Neutral Speed", INDEXER_STOP_NEUTRAL_SPEED);

        SmartDashboard.putNumber("Indexer Conveyer Intake Speed", INDEXER_CONVEYOR_INTAKE_SPEED);
        SmartDashboard.putNumber("Indexer Conveyer Shoot Speed", INDEXER_CONVEYOR_SHOOT_SPEED);
        SmartDashboard.putNumber("Indexer Conveyer Eject Speed", INDEXER_CONVEYOR_EJECT_SPEED);
        SmartDashboard.putNumber("Indexer Conveyer Neutral Speed", INDEXER_CONVEYOR_NEUTRAL_SPEED);
    }

    public void getConstantTuning() {
        if (INDEXER_PIDF[0] != SmartDashboard.getNumber("Indexer P", INDEXER_PIDF[0])) {
            INDEXER_PIDF[0] = SmartDashboard.getNumber("Indexer P", INDEXER_PIDF[0]);
            PIDconveyorTop.setP(INDEXER_PIDF[0]);
        }
        if (INDEXER_PIDF[1] != SmartDashboard.getNumber("Indexer I", INDEXER_PIDF[1])) {
            INDEXER_PIDF[1] = SmartDashboard.getNumber("Indexer I", INDEXER_PIDF[1]);
            PIDconveyorTop.setP(INDEXER_PIDF[1]);
        }
        if (INDEXER_PIDF[2] != SmartDashboard.getNumber("Indexer D", INDEXER_PIDF[2])) {
            INDEXER_PIDF[2] = SmartDashboard.getNumber( "Indexer D", INDEXER_PIDF[2]);
            PIDconveyorTop.setP(INDEXER_PIDF[2]);
        }
        if (INDEXER_PIDF[3] != SmartDashboard.getNumber("Indexer F", INDEXER_PIDF[3])) {
            INDEXER_PIDF[3] = SmartDashboard.getNumber("Indexer F", INDEXER_PIDF[3]);
            PIDconveyorTop.setP(INDEXER_PIDF[3]);
        }
        
        if (ONE_INDEX_SETPOINT != SmartDashboard.getNumber("Indexer One Setpoint", ONE_INDEX_SETPOINT)) {
            ONE_INDEX_SETPOINT = SmartDashboard.getNumber("Indexer One Setpoint", ONE_INDEX_SETPOINT);
        }

        if (INDEXER_STOP_SHOOT_SPEED != SmartDashboard.getNumber("Indexer Stop Shoot Speed", INDEXER_STOP_SHOOT_SPEED)) {
            INDEXER_STOP_SHOOT_SPEED = SmartDashboard.getNumber("Indexer Stop Shoot Speed", INDEXER_STOP_SHOOT_SPEED);
        }
        if (INDEXER_STOP_NEUTRAL_SPEED != SmartDashboard.getNumber("Indexer Stop Neutral Speed", INDEXER_STOP_NEUTRAL_SPEED)) {
            INDEXER_STOP_NEUTRAL_SPEED = SmartDashboard.getNumber("Indexer Stop Neutral Speed", INDEXER_STOP_NEUTRAL_SPEED);
        }

        if (INDEXER_CONVEYOR_SHOOT_SPEED != SmartDashboard.getNumber("Indexer Conveyer Shoot Speed", INDEXER_CONVEYOR_SHOOT_SPEED)) {
            INDEXER_CONVEYOR_SHOOT_SPEED = SmartDashboard.getNumber("Indexer Conveyer Shoot Speed", INDEXER_CONVEYOR_SHOOT_SPEED);
        }
        if (INDEXER_CONVEYOR_EJECT_SPEED != SmartDashboard.getNumber("Indexer Conveyer Eject Speed", INDEXER_CONVEYOR_EJECT_SPEED)) {
            INDEXER_CONVEYOR_EJECT_SPEED = SmartDashboard.getNumber("Indexer Conveyer Eject Speed", INDEXER_CONVEYOR_EJECT_SPEED);
        }
        if (INDEXER_CONVEYOR_INTAKE_SPEED != SmartDashboard.getNumber("Indexer Conveyer Intake Speed", INDEXER_CONVEYOR_INTAKE_SPEED)) {
            INDEXER_CONVEYOR_INTAKE_SPEED = SmartDashboard.getNumber("Indexer Conveyer Intake Speed", INDEXER_CONVEYOR_INTAKE_SPEED);
        }
        if (INDEXER_CONVEYOR_NEUTRAL_SPEED != SmartDashboard.getNumber("Indexer Conveyer Neutral Speed", INDEXER_CONVEYOR_NEUTRAL_SPEED)) {
            INDEXER_CONVEYOR_NEUTRAL_SPEED = SmartDashboard.getNumber("Indexer Conveyer Neutral Speed", INDEXER_CONVEYOR_NEUTRAL_SPEED);
        }
    }
     
    public double getEncoderVal() {
        return conveyorEncoderTop.getPosition();
    }

    public void resetEncoder() {
        conveyorEncoderTop.setPosition(RESET_SETPOINT);
    }

    public double getcurrentPIDSetpoint( ){
        return currentPIDSetpoint;
    }

    public void setcurrentPIDSetpoint(double value) {
        resetEncoder();
        currentPIDSetpoint = value;
        state = State.PID;
    }

    public void moveOneIndex() {
        setcurrentPIDSetpoint(ONE_INDEX_SETPOINT);
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
