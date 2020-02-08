package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends SnailSubsystem {

    private CANSparkMax conveyorMotorTop;
    private CANEncoder conveyorEncoder;
    private CANPIDController conveyorPID;

    private CANSparkMax conveyorMotorBottom;

    private CANSparkMax stopMotor;

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
        stopMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);

        currentPIDSetpoint = -1257.0;
    }
    
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

        SmartDashboard.putNumber("Conveyor Top Current", conveyorMotorTop.getOutputCurrent());
        SmartDashboard.putNumber("Top Encoder", getEncoderValue());

        SmartDashboard.putNumber("Conveyor Bottom Current", conveyorMotorBottom.getOutputCurrent());
        SmartDashboard.putNumber("Bottom Encoder", getEncoderValue());

        SmartDashboard.putNumber("Stop Motor Current", stopMotor.getOutputCurrent());

    }
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
     
    public double getEncoderValue() {
        return conveyorEncoder.getPosition();
    }

    public void resetEncoder() {
        conveyorEncoder.setPosition(0);
    }

    public void setCurrentPIDSetpoint(double value) {
        resetEncoder();
        currentPIDSetpoint = value;
        state = State.PID;
    }

    public void advance() {
        setCurrentPIDSetpoint(INDEXER_ADVANCE_SETPOINT);
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

    public State getState(){
        return state;
    }
}
