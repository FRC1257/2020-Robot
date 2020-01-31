package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Intake extends SubsystemBase {

    CANSparkMax intakeMotor;

    public enum State {
        NEUTRAL,
        INTAKING,
        EJECTING
    }
    State state = State.NEUTRAL;

    public Intake() {
       intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
       intakeMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMITER);

    }
    
    @Override
    public void periodic() {
        switch(state) {
            case NEUTRAL: 
                intakeMotor.set(INTAKE_NEUTRAL_SPEED);
                break;
            case INTAKING:
                intakeMotor.set(INTAKE_INTAKE_SPEED);
                break;
            case EJECTING:
                intakeMotor.set(INTAKE_EJECT_SPEED);
                break;
        }
    }
    public void outputValues() {
        SmartDashboard.putString("Intake State", state.name());

        SmartDashboard.putNumber("intake motor Current", intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("intake motor Temperature (C)", intakeMotor.getMotorTemperature());
    }

    public void neutral() {
        state = State.NEUTRAL;
    
    }
    public void eject() {
        state = State.EJECTING;
    }

    public void intake() {
        state = State.INTAKING;
    }
    public State getState(){
        return state;
    }
}
