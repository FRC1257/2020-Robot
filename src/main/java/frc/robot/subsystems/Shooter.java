package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private CANSparkMax shooterMotor;
    
    public enum State {
        NEUTRAL,
        SHOOTING
    }
    private State state = State.NEUTRAL;

    public Shooter() {
        shooterMotor = new CANSparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);
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
        }
    }

    public void neutral() {
        state = State.NEUTRAL;
    }

    public void shooting() {
        state = State.SHOOTING;
    }
}