package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX; // can change depending on what motor we're using
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// possibly the motor library import needed depending on what motor we're using

public class Shooter extends SubsystemBase {

    private WPI_VictorSPX shooterMotor;
    
    public enum State {
        NEUTRAL,
        SHOOTING
    }
    private State state = State.NEUTRAL;

    public Shooter() {
        shooterMotor = new WPI_VictorSPX(SHOOTER_MOTOR_ID);
    }

    @Override
    public void periodic() {
        switch(state) {
            case NEUTRAL:
                shooterMotor.set(NEUTRAL_SHOOTER_MOTOR_SPEED);
            case SHOOTING:
                shooterMotor.set(SHOOTING_SHOOTER_MOTOR_SPEED);
        }
    }

    public void neutral() {
        state = State.NEUTRAL;
    }

    public void shooting() {
        state = State.SHOOTING;
    }
}