package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import static frc.robot.Constants.ElectricalLayout;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.NEO_CURRENT_LIMIT;

public class Shooter extends SnailSubsystem {

    private final CANSparkMax shooterMotor;
    private final CANSparkMax followerMotor;

    private final CANPIDController shooterPID;
    private final CANEncoder shooterEncoder;
    
    public enum State {
        NEUTRAL,
        OPEN_LOOP,
        VEL_PID,
        BACKING
    }
    private State state = State.NEUTRAL;

    public Shooter() {
        shooterMotor = new CANSparkMax(ElectricalLayout.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(IdleMode.kCoast);
        shooterMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        followerMotor = new CANSparkMax(ElectricalLayout.SHOOTER_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        followerMotor.restoreFactoryDefaults();
        followerMotor.setIdleMode(IdleMode.kCoast);
        followerMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        followerMotor.follow(shooterMotor, true); // follow with inverted

        shooterPID = shooterMotor.getPIDController();
        shooterPID.setP(Constants.Shooter.SHOOTER_PIDF[0]);
        shooterPID.setI(Constants.Shooter.SHOOTER_PIDF[1]);
        shooterPID.setD(Constants.Shooter.SHOOTER_PIDF[2]);
        shooterPID.setFF(Constants.Shooter.SHOOTER_PIDF[3]);
        shooterEncoder = shooterMotor.getEncoder();
    }

    @Override
    public void update() {
        switch(state) {
            case NEUTRAL:
                shooterMotor.set(Constants.Shooter.SHOOTER_NEUTRAL_SPEED);
                break;
            case OPEN_LOOP:
                shooterMotor.set(Constants.Shooter.SHOOTER_OPEN_LOOP_SPEED);
                break;
            case VEL_PID:
                shooterPID.setReference(Constants.Shooter.SHOOTER_VEL_SETPOINT, ControlType.kVelocity);
                break;
            case BACKING:
                shooterMotor.set(Constants.Shooter.SHOOTER_BACK_SPEED);
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

    public void back() {
        state = State.BACKING;
    }

    public boolean withinTolerance() {
        return (Constants.Shooter.SHOOTER_VEL_SETPOINT - shooterEncoder.getVelocity()) / Constants.Shooter.SHOOTER_VEL_SETPOINT < Constants.Shooter.SHOOTER_PERCENT_TOLERANCE;
    }

    @Override
    public void outputValues() {
        if(SmartDashboard.getBoolean("Testing", false)) {
            SmartDashboard.putNumber("Shooter Encoder Vel", shooterEncoder.getVelocity());
            SmartDashboard.putString("Shooter State", state.name());
        }
        
        SmartDashboard.putBoolean("Shooter Within Tolerance", withinTolerance());
        SmartDashboard.putNumberArray("Shooter Currents",
            new double[] {shooterMotor.getOutputCurrent(), followerMotor.getOutputCurrent()});
    }

    @Override
    public void setUpConstantTuning() {
        SmartDashboard.putNumber("Shooter PID kP", SHOOTER_PIDF[0]);
        SmartDashboard.putNumber("Shooter PID kI", SHOOTER_PIDF[1]);
        SmartDashboard.putNumber("Shooter PID kD", SHOOTER_PIDF[2]);
        SmartDashboard.putNumber("Shooter PID kFF", SHOOTER_PIDF[3]);

        SmartDashboard.putNumber("Shooter Open Loop Speed", Constants.Shooter.SHOOTER_OPEN_LOOP_SPEED);
        SmartDashboard.putNumber("Shooter Vel PID Setpoint", Constants.Shooter.SHOOTER_VEL_SETPOINT);
    }

    @Override
    public void getConstantTuning() {
        SHOOTER_PIDF[0] = SmartDashboard.getNumber("Shooter PID kP", SHOOTER_PIDF[0]);
        SHOOTER_PIDF[1] = SmartDashboard.getNumber("Shooter PID kI", SHOOTER_PIDF[1]);
        SHOOTER_PIDF[2] = SmartDashboard.getNumber("Shooter PID kD", SHOOTER_PIDF[2]);
        SHOOTER_PIDF[3] = SmartDashboard.getNumber("Shooter PID kFF", SHOOTER_PIDF[3]);

        Constants.Shooter.SHOOTER_OPEN_LOOP_SPEED = SmartDashboard.getNumber("Shooter Open Loop Speed", Constants.Shooter.SHOOTER_OPEN_LOOP_SPEED);
        Constants.Shooter.SHOOTER_VEL_SETPOINT = SmartDashboard.getNumber("Shooter Vel PID Setpoint", Constants.Shooter.SHOOTER_VEL_SETPOINT);
        
        if (shooterPID.getP() != Constants.Shooter.SHOOTER_PIDF[0]) {
            shooterPID.setP(Constants.Shooter.SHOOTER_PIDF[0]);
        }
        if (shooterPID.getI() != Constants.Shooter.SHOOTER_PIDF[1]) {
            shooterPID.setI(Constants.Shooter.SHOOTER_PIDF[1]);
        }
        if (shooterPID.getD() != Constants.Shooter.SHOOTER_PIDF[2]) {
            shooterPID.setD(Constants.Shooter.SHOOTER_PIDF[2]);
        }
        if (shooterPID.getFF() != Constants.Shooter.SHOOTER_PIDF[3]) {
            shooterPID.setFF(Constants.Shooter.SHOOTER_PIDF[3]);
        }
    }
}
