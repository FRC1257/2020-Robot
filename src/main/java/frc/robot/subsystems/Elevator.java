package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class Elevator extends SnailSubsystem {
  
    private CANSparkMax motor;
    private CANSparkMax followerMotor;
    private CANPIDController elevatorPID;
    private CANEncoder encoder;

    private Servo servo;

    private ElevatorFeedforward feedforward;
    private TrapezoidProfile profile;
    private Timer profileTimer;

    private double currentPIDSetpoint;

    public enum State {
        MANUAL,
        CLOSED_LOOP,
        PID,
        PROFILED
    }

    private State defaultState = State.MANUAL;
    private State state = defaultState;
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
        encoder.setPositionConversionFactor(ELEVATOR_CONV_FACTOR);
        encoder.setVelocityConversionFactor(ELEVATOR_CONV_FACTOR / 60.0);

        elevatorPID = motor.getPIDController();
        elevatorPID.setP(ELEVATOR_PID[0]);
        elevatorPID.setI(ELEVATOR_PID[1]);
        elevatorPID.setD(ELEVATOR_PID[2]);

        servo = new Servo(ELEVATOR_BRAKE_SERVO_ID);

        feedforward = new ElevatorFeedforward(ELEVATOR_KS, ELEVATOR_KG, ELEVATOR_KV, ELEVATOR_KA);

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
                case CLOSED_LOOP:
                    double error = speed - encoder.getVelocity();
                    motor.setVoltage(error * ELEVATOR_VEL_PID_KP + feedforward.calculate(speed));
                    break;
                case PROFILED:
                    if (profile == null) {
                        state = defaultState;
                        break;
                    }
                    TrapezoidProfile.State currentStateProf = profile.calculate(profileTimer.get());

                    double positionError = currentStateProf.position - encoder.getPosition();
                    double velocityError = currentStateProf.velocity - encoder.getVelocity();

                    motor.setVoltage(velocityError * ELEVATOR_VEL_PID_KP + 
                        feedforward.calculate(currentStateProf.velocity) + 
                        positionError * ELEVATOR_PROFILE_POS_KP);

                    if (profile.isFinished(profileTimer.get())) {
                        state = defaultState;
                        profileTimer.stop();
                        profile = null;
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
   
    @Override 
    public void outputValues() {
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Elevator Current", motor.getOutputCurrent());
        SmartDashboard.putBoolean("Elevator Locked", locked);
    }

    @Override
    public void setConstantTuning() {
        SmartDashboard.putNumber("Elevator PID kP", ELEVATOR_PID[0]);
        SmartDashboard.putNumber("Elevator PID kI", ELEVATOR_PID[1]);
        SmartDashboard.putNumber("Elevator PID kD", ELEVATOR_PID[2]);

        SmartDashboard.putNumber("Elevator Vel kP", ELEVATOR_VEL_PID_KP);
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

        ELEVATOR_VEL_PID_KP = SmartDashboard.getNumber("Elevator Vel kP", ELEVATOR_VEL_PID_KP);
    }
    
    public void toggleLock() {
        locked = !locked;
    }

    public void setElevatorSpeed(double speed) {
        this.speed = speed;
        state = State.MANUAL;
    }

    public void setElevatorSpeedClosedLoop(double speed) {
        this.speed = speed;
        state = State.CLOSED_LOOP;
    }

    public void raise() {
        encoder.setPosition(0);
        currentPIDSetpoint = ELEVATOR_SETPOINT;
        state = State.PID;
    }

    public void raiseProfiled() {
        encoder.setPosition(0);
        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(DRIVE_PROFILE_MAX_VEL, DRIVE_PROFILE_MAX_ACC),
            new TrapezoidProfile.State(ELEVATOR_SETPOINT, 0),
            new TrapezoidProfile.State(0, 0));
        profileTimer.reset();
        profileTimer.start();
        state = State.PROFILED;
    }
}
