package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// possibly the motor library import needed

public class Shooter extends SubsystemBase {

    private WPI_VictorSPX shooterMotor;
    
    public enum State {
        MANUAL,
        SHOOTING
    }
    private State state = State.MANUAL;

    public Shooter() {
        shooterMotor = new WPI_VictorSPX(SHOOTER_MOTOR_ID);
    }

    @Override
    public void periodic() {
        switch(state) {
            case MANUAL:
            case SHOOTING:
        }
    }

    public void neutral() {
        state = State.NEUTRAL;
    }

    public void shooting() {
        state = State.SHOOTING;
    }
}