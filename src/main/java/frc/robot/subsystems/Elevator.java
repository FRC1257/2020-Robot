package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
    private WPI_TalonSRX motor;

    public enum State {
        MANUAL
    }

    private State state = State.MANUAL;
    private double speed;

    public Elevator() {
        motor = new WPI_TalonSRX(ELEVATOR_MOTOR_ID);
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