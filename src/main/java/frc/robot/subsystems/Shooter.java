package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class Shooter extends SnailSubsystem {

    private final CANSparkMax shooterMotor;
    private final CANSparkMax followerMotor;

    private final CANPIDController shooterPID;
    private final CANEncoder shooterEncoder;
    
    public enum State {
        NEUTRAL,
        OPEN_LOOP,
        VEL_PID
    }
    private State state = State.NEUTRAL;

    public Shooter() {
        shooterMotor = new CANSparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(IdleMode.kCoast);
        shooterMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        followerMotor = new CANSparkMax(SHOOTER_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        followerMotor.restoreFactoryDefaults();
        followerMotor.setIdleMode(IdleMode.kCoast);
        followerMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        followerMotor.follow(shooterMotor, true); // follow with inverted

        shooterPID = shooterMotor.getPIDController();
        shooterEncoder = shooterMotor.getEncoder();
    }

    @Override
    public void periodic() {
        switch(state) {
            case NEUTRAL:
                shooterMotor.set(SHOOTER_NEUTRAL_SPEED);
                break;
            case OPEN_LOOP:
                shooterMotor.set(SHOOTER_OPEN_LOOP_SPEED);
                break;
            case VEL_PID:
                shooterPID.setReference(SHOOTER_VEL_SETPOINT, ControlType.kVelocity);
                break;
        }
    }

    public void neutral() {
        state = State.NEUTRAL;
    }

    public void openLoopShooting() {
        state = State.OPEN_LOOP;
    }

    public void velocityPIDShooting() {
        state = State.VEL_PID;
    }

    @Override
    public void outputValues() {
        SmartDashboard.putNumber("Shooter Encoder Pos", shooterEncoder.getPosition());
        SmartDashboard.putNumber("Shooter Encoder Vel", shooterEncoder.getVelocity());

        boolean inTolerance = (SHOOTER_VEL_SETPOINT - shooterEncoder.getVelocity()) / SHOOTER_VEL_SETPOINT < 0.03;
        SmartDashboard.putBoolean("Shooter Ready", inTolerance);

        SmartDashboard.putNumber("Shooter Primary Output", shooterMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter Secondary Output", followerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter Primary Current", shooterMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter Secondary Current", followerMotor.getOutputCurrent());
    }

    @Override
    public void setConstantTuning() {
        SmartDashboard.putNumber("Shooter PID kP", SHOOTER_PIDF[0]);
        SmartDashboard.putNumber("Shooter PID kI", SHOOTER_PIDF[1]);
        SmartDashboard.putNumber("Shooter PID kD", SHOOTER_PIDF[2]);
        SmartDashboard.putNumber("Shooter PID kFF", SHOOTER_PIDF[3]);

        SmartDashboard.putNumber("Shooter Open Loop Speed", SHOOTER_OPEN_LOOP_SPEED);
        SmartDashboard.putNumber("Shooter Vel PID Setpoint", SHOOTER_VEL_SETPOINT);
    }

    @Override
    public void getConstantTuning() {
        if (shooterPID.getP() != SmartDashboard.getNumber("Shooter PID kP", 0)) {
            shooterPID.setP(SmartDashboard.getNumber("Shooter PID kP", 0));
        }
        if (shooterPID.getI() != SmartDashboard.getNumber("Shooter PID kI", 0)) {
            shooterPID.setI(SmartDashboard.getNumber("Shooter PID kI", 0));
        }
        if (shooterPID.getD() != SmartDashboard.getNumber("Shooter PID kD", 0)) {
            shooterPID.setD(SmartDashboard.getNumber("Shooter PID kD", 0));
        }
        if (shooterPID.getFF() != SmartDashboard.getNumber("Shooter PID kFF", 0)) {
            shooterPID.setFF(SmartDashboard.getNumber("Shooter PID kFF", 0));
        }

        SHOOTER_OPEN_LOOP_SPEED = SmartDashboard.getNumber("Shooter Open Loop Speed", SHOOTER_OPEN_LOOP_SPEED);
        SHOOTER_VEL_SETPOINT = SmartDashboard.getNumber("Shooter Vel PID Setpoint", SHOOTER_VEL_SETPOINT);
    }
}
