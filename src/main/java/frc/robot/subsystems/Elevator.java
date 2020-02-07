package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
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
        motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);

        encoder = motor.getEncoder();

        elevatorPID = motor.getPIDController();
        elevatorPID.setP(ELEVATOR_PIDF[0]);
        elevatorPID.setI(ELEVATOR_PIDF[1]);
        elevatorPID.setD(ELEVATOR_PIDF[2]);
        elevatorPID.setFF(ELEVATOR_PIDF[3]);
        elevatorPID.setIZone(0);
        elevatorPID.setOutputRange(ELEVATOR_PID_MIN_OUTPUT, ELEVATOR_PID_MAX_OUTPUT);

        currentPIDSetpoint = -1257;
    }

    @Override
    public void periodic() {
        switch(state) {
            case MANUAL:
                motor.set(speed);
            break;
            case PID:
                if (currentPIDSetpoint != ELEVATOR_TOP) break;
                elevatorPID.setReference(currentPIDSetpoint, ControlType.kPosition);
            break;
        }
        speed = 0;
    }

    public void setElevatorSpeed(double speed) {
        this.speed = speed;
        state = State.MANUAL;
    }

    public void raise() {
        encoder.setPosition(0);
        currentPIDSetpoint = ELEVATOR_TOP;
        state = State.PID;
    }
}