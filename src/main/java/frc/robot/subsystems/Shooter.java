package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
        shooterPID = shooterMotor.getPIDController();

        SmartDashboard.putNumber("Shooter P", SHOOTER_PIDF[0]);
        SmartDashboard.putNumber("Shooter I", SHOOTER_PIDF[1]);
        SmartDashboard.putNumber("Shooter D", SHOOTER_PIDF[2]);
        SmartDashboard.putNumber("Shooter FF", SHOOTER_PIDF[3]);
        SmartDashboard.putNumber("Shooter Setpoint", SHOOTER_SETPOINT);
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
                SmartDashboard.putNumber("Shooter Encoder Pos", shooterEncoder.getPosition());
                SmartDashboard.putNumber("Shooter Encoder Vel", shooterEncoder.getVelocity());

                break;
        }
    }

    public void neutral() {
        state = State.NEUTRAL;
    }

    public void shooting() {
        state = State.SHOOTING;
    }
}