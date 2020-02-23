package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class Elevator extends SnailSubsystem {
  
    private final CANSparkMax motor;
    private final CANSparkMax followerMotor;
    private final CANPIDController elevatorPID;
    private final CANEncoder encoder;

    private final Servo servo;

    private double currentPIDSetpoint;

    public enum State {
        MANUAL,
        PID,
    }

    private State state = State.MANUAL;
    private double speed;
    private boolean locked;

    public Elevator() {
        motor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(true);

        followerMotor = new CANSparkMax(ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        followerMotor.restoreFactoryDefaults();
        followerMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        followerMotor.setIdleMode(IdleMode.kBrake);
        followerMotor.follow(motor);

        encoder = motor.getEncoder();

        elevatorPID = motor.getPIDController();
        elevatorPID.setP(ELEVATOR_PID[0]);
        elevatorPID.setI(ELEVATOR_PID[1]);
        elevatorPID.setD(ELEVATOR_PID[2]);

        servo = new Servo(ELEVATOR_BRAKE_SERVO_ID);

        reset();
    }

    private void reset() {
        speed = 0;
        encoder.setPosition(0);
        currentPIDSetpoint = -1257;
        locked = false;
    }

    @Override
    public void periodic() {
        if (!locked) {
            switch(state) {
                case MANUAL:
                    motor.set(speed);
                    break;
                case PID:
                    if (currentPIDSetpoint == -1257.0) {
                        break;
                    }

                    elevatorPID.setReference(currentPIDSetpoint, ControlType.kPosition);

                    if (Math.abs(encoder.getPosition() - currentPIDSetpoint) < ELEVATOR_PID_TOLERANCE) {
                        state = State.MANUAL;
                    }
                    break;
            }
            servo.set(0.0);
        }
        else {
            servo.set(ELEVATOR_BRAKE_POSITION);
        }
        
        speed = 0;
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

    public void toggleLock() {
        locked = !locked;
    }
   
    @Override 
    public void outputValues() {
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Current", motor.getOutputCurrent());
        SmartDashboard.putBoolean("Elevator Locked", locked);
    }

    @Override
    public void setConstantTuning() {
        SmartDashboard.putNumber("Elevator PID kP", ELEVATOR_PID[0]);
        SmartDashboard.putNumber("Elevator PID kI", ELEVATOR_PID[1]);
        SmartDashboard.putNumber("Elevator PID kD", ELEVATOR_PID[2]);
    }

    @Override
    public void getConstantTuning() {
        if (elevatorPID.getP() != SmartDashboard.getNumber("Elevator PID kP", ELEVATOR_PID[0])) {
            ELEVATOR_PID[0] = SmartDashboard.getNumber("Elevator PID kP", ELEVATOR_PID[0]);
            elevatorPID.setP(ELEVATOR_PID[0]);
        }
        if (elevatorPID.getI() != SmartDashboard.getNumber("Elevator PID kI", ELEVATOR_PID[1])) {
            ELEVATOR_PID[1] = SmartDashboard.getNumber("Elevator PID kI", ELEVATOR_PID[1]);
            elevatorPID.setI(ELEVATOR_PID[1]);
        }
        if (elevatorPID.getD() != SmartDashboard.getNumber("Elevator PID kD", ELEVATOR_PID[2])) {
            ELEVATOR_PID[2] = SmartDashboard.getNumber( "Elevator PID kD", ELEVATOR_PID[2]);
            elevatorPID.setD(ELEVATOR_PID[2]);
        }
    }
}
