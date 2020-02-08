package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends SnailSubsystem {
  
    private CANSparkMax motor;

    public enum State {
        MANUAL
    }

    private State state = State.MANUAL;
    private double speed;

    public Elevator() {
        motor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        switch(state) {
            case MANUAL:
                motor.set(speed);
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
    }
}