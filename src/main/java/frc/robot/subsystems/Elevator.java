package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends SnailSubsystem {
  
    private CANSparkMax motor;
    private CANPIDController elevatorPID;
    private CANEncoder encoder;

    private double currentPIDSetpoint;

    public enum State {
        MANUAL, PID;
    }

    private State state = State.MANUAL;
    private double speed;

    public Elevator() {
        motor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        motor.setIdleMode(IdleMode.kBrake);

        encoder = motor.getEncoder();

        elevatorPID = motor.getPIDController();
        elevatorPID.setP(ELEVATOR_PID[0]);
        elevatorPID.setI(ELEVATOR_PID[1]);
        elevatorPID.setD(ELEVATOR_PID[2]);

        reset();
    }

    private void reset() {
        speed = 0;
        encoder.setPosition(0);
        currentPIDSetpoint = -1257;
    }

    @Override
    public void periodic() {
        switch(state) {
            case MANUAL:
                motor.set(speed);
                break;
            case PID:
                if (currentPIDSetpoint == -1257.0) {
                    break;
                }
                elevatorPID.setReference(currentPIDSetpoint, ControlType.kPosition);
                break;
        }
        
        speed = 0;
    }
    
    public void outputValues() {
        
    }

    public void setConstantTuning() {
        
    }

    public void getConstantTuning() {
        
    }
    

    public void setElevatorSpeed(double speed) {
        this.speed = speed;
        state = State.MANUAL;
    }

    public void raise() {
        encoder.setPosition(0);
        currentPIDSetpoint = ELEVATOR_SETPOINT;
        state = State.PID;
    }
}
