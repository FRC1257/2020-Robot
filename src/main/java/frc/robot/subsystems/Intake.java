package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

class Intake extends SubsystemBase {

    CANSparkMax intakeMotor;

    public enum State {
        INTAKE,
        NEUTRAL,
        EJECT
    }
    State state = State.NEUTRAL;

    public Intake() {
       intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    }
    
    @Override
    public void periodic() {
        switch(state) {
            case NEUTRAL: 
                intakeMotor.set(NEUTRAL_SPEED);
                break;
            case INTAKE:
                intakeMotor.set(INTAKE_SPEED);
                break;
            case EJECT:
                intakeMotor.set(EJECT_SPEED);
                break;
        }
    }
}