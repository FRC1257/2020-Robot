package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
    private CANSparkMax motor;

    public enum State {
        MANUAL
    }

    private State state = State.MANUAL;
    private double speed;

    public Elevator() {
        motor = new CANSparkMax(ELEVATOR_MOTOR_ID,MotorType.kBrushless);
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
    
    public void setElevatorSpeed(double speed) {
        this.speed = speed;
    }
}