package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private CANSparkMax shooterMotor;
    private CANPIDController shooterPID;
    private CANEncoder shooterEncoder;
    
    public enum State {
        NEUTRAL,
        SHOOTING,
        PID
    }
    private State state = State.NEUTRAL;

    public Shooter() {
        shooterMotor = new CANSparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(IdleMode.kCoast);

        shooterPID = shooterMotor.getPIDController();
        shooterEncoder = shooterMotor.getEncoder();

        setConstantTuning();
    }

    @Override
    public void periodic() {
        switch(state) {
            case NEUTRAL:
                shooterMotor.set(NEUTRAL_SHOOTER_MOTOR_SPEED);
                break;
            case SHOOTING:
                shooterMotor.set(SHOOTING_SHOOTER_MOTOR_SPEED);
                break;
            case PID:
                shooterPID.setReference(SHOOTER_SETPOINT, ControlType.kVelocity);
                break;
        }
    }

    public void neutral() {
        state = State.NEUTRAL;
    }

    public void shooting() {
        state = State.SHOOTING;
    }

    public void pid() {
        state = State.PID;
    }

    public void outputValues() {
        SmartDashboard.putNumber("Shooter Encoder Pos", shooterEncoder.getPosition());
        SmartDashboard.putNumber("Shooter Encoder Vel", shooterEncoder.getVelocity());
    }

    public void setConstantTuning() {
        SmartDashboard.putNumber("Shooter PID kP", SHOOTER_PIDF[0]);
        SmartDashboard.putNumber("Shooter PID kI", SHOOTER_PIDF[1]);
        SmartDashboard.putNumber("Shooter PID kD", SHOOTER_PIDF[2]);
        SmartDashboard.putNumber("Shooter PID kFF", SHOOTER_PIDF[3]);
        SmartDashboard.putNumber("Shooter PID Setpoint", SHOOTER_SETPOINT);
    }

    public void getConstantTuning() {
        if(shooterPID.getP() != SmartDashboard.getNumber("Shooter PID kP", 0)) {
            shooterPID.setP(SmartDashboard.getNumber("Shooter PID kP", 0));
        }
        if(shooterPID.getI() != SmartDashboard.getNumber("Shooter PID kI", 0)) {
            shooterPID.setI(SmartDashboard.getNumber("Shooter PID kI", 0));
        }
        if(shooterPID.getD() != SmartDashboard.getNumber("Shooter PID kD", 0)) {
            shooterPID.setD(SmartDashboard.getNumber("Shooter PID kD", 0));
        }
        if(shooterPID.getFF() != SmartDashboard.getNumber("Shooter PID kFF", 0)) {
            shooterPID.setFF(SmartDashboard.getNumber("Shooter PID kFF", 0));
        }

        SHOOTER_SETPOINT = SmartDashboard.getNumber("Shooter PID Setpoint", SHOOTER_SETPOINT);
    }
}
