package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import static frc.robot.Constants.ElectricalLayout;
import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.NEO_CURRENT_LIMIT;

// import edu.wpi.first.wpilibj.Servo;

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
        motor = new CANSparkMax(ElectricalLayout.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(true);

        followerMotor = new CANSparkMax(ElectricalLayout.ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        followerMotor.restoreFactoryDefaults();
        followerMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        followerMotor.setIdleMode(IdleMode.kBrake);
        followerMotor.follow(motor, false);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(Constants.Elevator.ELEVATOR_CONV_FACTOR);
        encoder.setVelocityConversionFactor(Constants.Elevator.ELEVATOR_CONV_FACTOR / 60.0);

        elevatorPID = motor.getPIDController();
        elevatorPID.setP(Constants.Elevator.ELEVATOR_PID[0], Constants.Elevator.ELEVATOR_PID_SLOT_POS);
        elevatorPID.setI(Constants.Elevator.ELEVATOR_PID[1], Constants.Elevator.ELEVATOR_PID_SLOT_POS);
        elevatorPID.setD(Constants.Elevator.ELEVATOR_PID[2], Constants.Elevator.ELEVATOR_PID_SLOT_POS);

        elevatorPID.setP(Constants.Elevator.ELEVATOR_VEL_PIF[0], Constants.Elevator.ELEVATOR_PID_SLOT_VEL);
        elevatorPID.setI(Constants.Elevator.ELEVATOR_VEL_PIF[1], Constants.Elevator.ELEVATOR_PID_SLOT_VEL);
        elevatorPID.setFF(Constants.Elevator.ELEVATOR_VEL_PIF[2], Constants.Elevator.ELEVATOR_PID_SLOT_VEL);
        elevatorPID.setSmartMotionMaxVelocity(Constants.Elevator.ELEVATOR_PROFILE_MAX_VEL, Constants.Elevator.ELEVATOR_PID_SLOT_VEL);
        elevatorPID.setSmartMotionMaxAccel(Constants.Elevator.ELEVATOR_PROFILE_MAX_ACC, Constants.Elevator.ELEVATOR_PID_SLOT_VEL);

        // servo = new Servo(ELEVATOR_BRAKE_SERVO_ID);

        feedforward = new ElevatorFeedforward(Constants.Elevator.ELEVATOR_KS, Constants.Elevator.ELEVATOR_KG, Constants.Elevator.ELEVATOR_KV, Constants.Elevator.ELEVATOR_KA);

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
            if (speed > 0 && encoder.getPosition() >= Constants.Elevator.ELEVATOR_MAX_HEIGHT && !override) {
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

                    elevatorPID.setReference(setpoint, ControlType.kPosition, Constants.Elevator.ELEVATOR_PID_SLOT_POS);
                    setpoint = -1257;
                    break;
                case CLOSED_LOOP:
                    elevatorPID.setReference(speed, ControlType.kVelocity, Constants.Elevator.ELEVATOR_PID_SLOT_VEL,
                            feedforward.calculate(0), CANPIDController.ArbFFUnits.kVoltage);
                    break;
                case PROFILED:
                    if (setpoint == -1257) {
                        state = defaultState;
                        break;
                    }

                    elevatorPID.setReference(setpoint, ControlType.kSmartMotion, Constants.Elevator.ELEVATOR_PID_SLOT_VEL,
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
        setpoint = Constants.Elevator.ELEVATOR_SETPOINT;
        state = State.PID;
    }

    public void raiseProfiled() {
        encoder.setPosition(0);
        setpoint = Constants.Elevator.ELEVATOR_SETPOINT;
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
        SmartDashboard.putNumber("Elevator PID kP", ELEVATOR_PID[0]);
        SmartDashboard.putNumber("Elevator PID kI", ELEVATOR_PID[1]);
        SmartDashboard.putNumber("Elevator PID kD", ELEVATOR_PID[2]);

        SmartDashboard.putNumber("Elevator Vel kP", ELEVATOR_VEL_PIF[0]);
        SmartDashboard.putNumber("Elevator Vel kI", ELEVATOR_VEL_PIF[1]);
        SmartDashboard.putNumber("Elevator Vel kFF", ELEVATOR_VEL_PIF[2]);

        SmartDashboard.putNumber("Elevator PID Setpoint", ELEVATOR_SETPOINT);
    }

    @Override
    public void getConstantTuning() {
        ELEVATOR_PID[0] = SmartDashboard.getNumber("Elevator PID kP", ELEVATOR_PID[0]);
        ELEVATOR_PID[1] = SmartDashboard.getNumber("Elevator PID kI", ELEVATOR_PID[1]);
        ELEVATOR_PID[2] = SmartDashboard.getNumber("Elevator PID kD", ELEVATOR_PID[2]);

        ELEVATOR_VEL_PIF[0] = SmartDashboard.getNumber("Elevator Vel kP", ELEVATOR_VEL_PIF[0]);
        ELEVATOR_VEL_PIF[1] = SmartDashboard.getNumber("Elevator Vel kI", ELEVATOR_VEL_PIF[1]);
        ELEVATOR_VEL_PIF[2] = SmartDashboard.getNumber("Elevator Vel kFF", ELEVATOR_VEL_PIF[2]);

        ELEVATOR_SETPOINT = SmartDashboard.getNumber("Elevator PID Setpoint", ELEVATOR_SETPOINT);

        if (elevatorPID.getP(Constants.Elevator.ELEVATOR_PID_SLOT_POS) != Constants.Elevator.ELEVATOR_PID[0]) {
            elevatorPID.setP(Constants.Elevator.ELEVATOR_PID[0], Constants.Elevator.ELEVATOR_PID_SLOT_POS);
        }
        if (elevatorPID.getI(Constants.Elevator.ELEVATOR_PID_SLOT_POS) != Constants.Elevator.ELEVATOR_PID[1]) {
            elevatorPID.setI(Constants.Elevator.ELEVATOR_PID[1], Constants.Elevator.ELEVATOR_PID_SLOT_POS);
        }
        if (elevatorPID.getD(Constants.Elevator.ELEVATOR_PID_SLOT_POS) != Constants.Elevator.ELEVATOR_PID[2]) {
            elevatorPID.setD(Constants.Elevator.ELEVATOR_PID[2], Constants.Elevator.ELEVATOR_PID_SLOT_POS);
        }

        if (elevatorPID.getP(Constants.Elevator.ELEVATOR_PID_SLOT_VEL) != Constants.Elevator.ELEVATOR_VEL_PIF[0]) {
            elevatorPID.setP(Constants.Elevator.ELEVATOR_VEL_PIF[0], Constants.Elevator.ELEVATOR_PID_SLOT_VEL);
        }
        if (elevatorPID.getI(Constants.Elevator.ELEVATOR_PID_SLOT_VEL) != Constants.Elevator.ELEVATOR_VEL_PIF[1]) {
            elevatorPID.setI(Constants.Elevator.ELEVATOR_VEL_PIF[1], Constants.Elevator.ELEVATOR_PID_SLOT_VEL);
        }
        if (elevatorPID.getFF(Constants.Elevator.ELEVATOR_PID_SLOT_VEL) != Constants.Elevator.ELEVATOR_VEL_PIF[2]) {
            elevatorPID.setFF(Constants.Elevator.ELEVATOR_VEL_PIF[2], Constants.Elevator.ELEVATOR_PID_SLOT_VEL);
        }
    }
}
