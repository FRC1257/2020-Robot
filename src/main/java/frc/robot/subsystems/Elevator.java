package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
// import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class Elevator extends SnailSubsystem {
  
    private final CANSparkMax motor;
    private final CANSparkMax followerMotor;
    private final CANPIDController elevatorPID;
    private final CANEncoder encoder;

    // private final Servo servo;

    private ElevatorFeedforward feedforward;

    private double setpoint;

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
    private boolean override;

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
        followerMotor.follow(motor, false);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(ELEVATOR_CONV_FACTOR);
        encoder.setVelocityConversionFactor(ELEVATOR_CONV_FACTOR / 60.0);

        elevatorPID = motor.getPIDController();
        elevatorPID.setP(ELEVATOR_PID[0], ELEVATOR_PID_SLOT_POS);
        elevatorPID.setI(ELEVATOR_PID[1], ELEVATOR_PID_SLOT_POS);
        elevatorPID.setD(ELEVATOR_PID[2], ELEVATOR_PID_SLOT_POS);

        elevatorPID.setP(ELEVATOR_VEL_PIF[0], ELEVATOR_PID_SLOT_VEL);
        elevatorPID.setI(ELEVATOR_VEL_PIF[1], ELEVATOR_PID_SLOT_VEL);
        elevatorPID.setFF(ELEVATOR_VEL_PIF[2], ELEVATOR_PID_SLOT_VEL);
        elevatorPID.setSmartMotionMaxVelocity(ELEVATOR_PROFILE_MAX_VEL, ELEVATOR_PID_SLOT_VEL);
        elevatorPID.setSmartMotionMaxAccel(ELEVATOR_PROFILE_MAX_ACC, ELEVATOR_PID_SLOT_VEL);

        // servo = new Servo(ELEVATOR_BRAKE_SERVO_ID);

        feedforward = new ElevatorFeedforward(ELEVATOR_KS, ELEVATOR_KG, ELEVATOR_KV, ELEVATOR_KA);

        reset();
    }

    private void reset() {
        speed = 0;
        encoder.setPosition(0);
        setpoint = -1257;
        locked = false;
        override = false;
    }

    @Override
    public void periodic() {
        // if (!locked) {
            if (speed > 0 && encoder.getPosition() >= ELEVATOR_MAX_HEIGHT && !override) {
                speed = 0;
            }
            if (speed < 0 && encoder.getPosition() <= 0 && !override) {
                speed = 0;
            }

            switch(state) {
                case MANUAL:
                    motor.set(speed);
                    setpoint = -1257;
                    break;
                case PID:
                    if (setpoint == -1257.0) {
                        break;
                    }

                    elevatorPID.setReference(setpoint, ControlType.kPosition, ELEVATOR_PID_SLOT_POS);
                    setpoint = -1257;
                    break;
                case CLOSED_LOOP:
                    elevatorPID.setReference(speed, ControlType.kVelocity, ELEVATOR_PID_SLOT_VEL,
                            feedforward.calculate(0), CANPIDController.ArbFFUnits.kVoltage);
                    break;
                case PROFILED:
                    if (setpoint == -1257) {
                        state = defaultState;
                        break;
                    }

                    elevatorPID.setReference(setpoint, ControlType.kSmartMotion, ELEVATOR_PID_SLOT_VEL,
                            feedforward.calculate(0), CANPIDController.ArbFFUnits.kVoltage);
                    break;
            }
            // servo.set(0.0);
        // }
        // else {
            // servo.set(ELEVATOR_BRAKE_POSITION);
        // }
        
        speed = 0;
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
        setpoint = ELEVATOR_SETPOINT;
        state = State.PID;
    }

    public void raiseProfiled() {
        encoder.setPosition(0);
        setpoint = ELEVATOR_SETPOINT;
        state = State.PROFILED;
    }

    public void setOverride(boolean override) {
        this.override = override;
    }
   
    @Override 
    public void outputValues() {
        SmartDashboard.putNumberArray("Elevator Stats (pos, vel, curr, temp)", new double[] {
            encoder.getPosition(),
            encoder.getVelocity(),
            motor.getOutputCurrent(),
            motor.getMotorTemperature()
        });

        SmartDashboard.putBoolean("Elevator Locked", locked);
    }

    @Override
    public void setConstantTuning() {
        SmartDashboard.putNumberArray("Elevator PID (kP, kI, kD)", ELEVATOR_PID);
        SmartDashboard.putNumberArray("Elevator Vel (kP, kI, kF)", ELEVATOR_VEL_PIF);
        SmartDashboard.putNumber("Elevator PID Setpoint", ELEVATOR_SETPOINT);
    }

    @Override
    public void getConstantTuning() {
        ELEVATOR_PID = SmartDashboard.getNumberArray("Elevator PID (kP, kI, kD)", ELEVATOR_PID);
        ELEVATOR_VEL_PIF = SmartDashboard.getNumberArray("Elevator Vel (kP, kI, kF)", ELEVATOR_VEL_PIF);
        ELEVATOR_SETPOINT = SmartDashboard.getNumber("Elevator PID Setpoint", ELEVATOR_SETPOINT);

        if (elevatorPID.getP(ELEVATOR_PID_SLOT_POS) != ELEVATOR_PID[0]) {
            elevatorPID.setP(ELEVATOR_PID[0], ELEVATOR_PID_SLOT_POS);
        }
        if (elevatorPID.getI(ELEVATOR_PID_SLOT_POS) != ELEVATOR_PID[1]) {
            elevatorPID.setI(ELEVATOR_PID[1], ELEVATOR_PID_SLOT_POS);
        }
        if (elevatorPID.getD(ELEVATOR_PID_SLOT_POS) != ELEVATOR_PID[2]) {
            elevatorPID.setD(ELEVATOR_PID[2], ELEVATOR_PID_SLOT_POS);
        }

        if (elevatorPID.getP(ELEVATOR_PID_SLOT_VEL) != ELEVATOR_VEL_PIF[0]) {
            elevatorPID.setP(ELEVATOR_VEL_PIF[0], ELEVATOR_PID_SLOT_VEL);
        }
        if (elevatorPID.getI(ELEVATOR_PID_SLOT_VEL) != ELEVATOR_VEL_PIF[1]) {
            elevatorPID.setI(ELEVATOR_VEL_PIF[1], ELEVATOR_PID_SLOT_VEL);
        }
        if (elevatorPID.getFF(ELEVATOR_PID_SLOT_VEL) != ELEVATOR_VEL_PIF[2]) {
            elevatorPID.setFF(ELEVATOR_VEL_PIF[2], ELEVATOR_PID_SLOT_VEL);
        }
    }
}
