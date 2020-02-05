package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
    private CANSparkMax motor;
    private CANPIDController elevatorPID;

    public enum State {
        MANUAL
    }

    private State state = State.MANUAL;
    private double speed;

    public Elevator() {
        motor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        elevatorPID = motor.getPIDController(); // TODO actually implement PID
        elevatorPID.setP(ELEVATOR_PIDF[0]);
        elevatorPID.setI(ELEVATOR_PIDF[1]);
        elevatorPID.setD(ELEVATOR_PIDF[2]);
        elevatorPID.setFF(ELEVATOR_PIDF[3]);
        elevatorPID.setIZone(0);
        elevatorPID.setOutputRange(ELEVATOR_PID_MIN_OUTPUT, ELEVATOR_PID_MAX_OUTPUT);
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