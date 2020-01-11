package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Gyro;

public class Drivetrain extends SubsystemBase {
  
    private CANSparkMax frontLeftMotor;
    private CANSparkMax frontRightMotor;
    private CANSparkMax backLeftMotor;
    private CANSparkMax backRightMotor;

    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private PIDController leftVelPID;
    private PIDController rightVelPID;

    private SimpleMotorFeedforward feedforward;
    private DifferentialDriveKinematics driveKinematics;
    private DifferentialDriveVoltageConstraint voltageConstraint;
    private TrajectoryConfig trajectoryConfig;
    private DifferentialDriveOdometry driveOdometry;

    private PIDController distPID;
    private PIDController anglePID;
    
    private double distSetpoint;
    private double angleSetpoint;

    private TrapezoidProfile distProfile;
    private Timer profileTimer;

    private Trajectory trajectory;

    public enum State {
        MANUAL,
        CLOSED_LOOP,
        PID_DIST,
        PID_ANGLE,
        PROFILE_DIST
    }
    private State defaultState = State.MANUAL;
    private State state = defaultState;

    private double speedForward;
    private double speedTurn;

    public Drivetrain() {
        frontLeftMotor = new CANSparkMax(DRIVE_FRONT_LEFT, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(DRIVE_FRONT_RIGHT, MotorType.kBrushless);
        backLeftMotor = new CANSparkMax(DRIVE_BACK_LEFT, MotorType.kBrushless);
        backRightMotor = new CANSparkMax(DRIVE_BACK_RIGHT, MotorType.kBrushless);

        frontLeftMotor.restoreFactoryDefaults();
        frontRightMotor.restoreFactoryDefaults();
        backLeftMotor.restoreFactoryDefaults();
        backRightMotor.restoreFactoryDefaults();

        frontLeftMotor.setIdleMode(IdleMode.kCoast);
        frontRightMotor.setIdleMode(IdleMode.kCoast);
        backLeftMotor.setIdleMode(IdleMode.kCoast);
        backRightMotor.setIdleMode(IdleMode.kCoast);

        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);

        leftEncoder = new CANEncoder(frontLeftMotor);
        rightEncoder = new CANEncoder(frontRightMotor);
        leftEncoder.setPositionConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M);
        rightEncoder.setPositionConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M);
        leftEncoder.setVelocityConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / 60.0);
        rightEncoder.setVelocityConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / 60.0);

        leftVelPID = new PIDController(DRIVE_LEFT_VEL_PID_P, 0, 0);
        rightVelPID = new PIDController(DRIVE_RIGHT_VEL_PID_P, 0, 0);

        feedforward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);
        driveKinematics = new DifferentialDriveKinematics(DRIVE_TRACK_WIDTH_M);
        voltageConstraint = new DifferentialDriveVoltageConstraint(feedforward, driveKinematics, DRIVE_MAX_VOLTAGE_RAMSETE);
        trajectoryConfig = new TrajectoryConfig(DRIVE_RAMSETE_MAX_VEL, DRIVE_RAMSETE_MAX_VEL)
            .setKinematics(driveKinematics)
            .addConstraint(voltageConstraint);
        driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(Gyro.getInstance().getRobotAngle()));

        distPID = new PIDController(DRIVE_DIST_PID_P, DRIVE_DIST_PID_I, DRIVE_DIST_PID_D);
        distPID.setTolerance(DRIVE_DIST_PID_TOLERANCE);
        
        anglePID = new PIDController(DRIVE_ANGLE_PID_P, DRIVE_ANGLE_PID_I, DRIVE_ANGLE_PID_D);
        anglePID.setTolerance(DRIVE_ANGLE_PID_TOLERANCE);
        anglePID.enableContinuousInput(-180.0, 180.0);

        profileTimer = new Timer();

        reset();
    }

    public void reset() {
        state = defaultState;
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        distSetpoint = -1257;
        angleSetpoint = -1257;
        profileTimer.stop();
    }

    @Override
    public void periodic() {
        switch(state) {
            case MANUAL:
                double[] arcadeSpeeds = arcadeDrive(speedForward, speedTurn);

                frontLeftMotor.set(arcadeSpeeds[0]);
                frontRightMotor.set(arcadeSpeeds[1]);
            break;
            case CLOSED_LOOP:
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speedForward, 0, speedTurn);
                DifferentialDriveWheelSpeeds dSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);

                frontLeftMotor.setVoltage(leftVelPID.calculate(leftEncoder.getVelocity(), dSpeeds.leftMetersPerSecond) + 
                    feedforward.calculate(dSpeeds.leftMetersPerSecond));
                frontRightMotor.setVoltage(rightVelPID.calculate(rightEncoder.getVelocity(), dSpeeds.rightMetersPerSecond) + 
                    feedforward.calculate(dSpeeds.leftMetersPerSecond));
            break;
            case PID_DIST:
                if(distSetpoint == -1257) {
                    state = defaultState;
                }
                
                // Apply PID controller to forward speed and basic P control to angle
                double[] pidDistArcadeSpeeds = arcadeDrive(distPID.calculate(leftEncoder.getPosition(), distSetpoint),
                    -Gyro.getInstance().getRobotAngle() * DRIVE_MAINTAIN_ANGLE_PID_P);

                frontLeftMotor.set(pidDistArcadeSpeeds[0]);
                frontRightMotor.set(pidDistArcadeSpeeds[1]);

                if(distPID.atSetpoint()) {
                    state = defaultState;
                    distSetpoint = -1257;
                }
            break;
            case PID_ANGLE:
                if(angleSetpoint == -1257) {
                    state = defaultState;
                }

                // Apply PID controller to forward speed and basic P control to angle
                double[] pidAngleArcadeSpeeds = arcadeDrive(0, 
                    anglePID.calculate(Gyro.getInstance().getRobotAngle(), angleSetpoint));

                frontLeftMotor.set(pidAngleArcadeSpeeds[0]);
                frontRightMotor.set(pidAngleArcadeSpeeds[1]);

                if(anglePID.atSetpoint()) {
                    state = defaultState;
                    angleSetpoint = -1257;
                }
            break;
            case PROFILE_DIST:
                if(distProfile == null) {
                    state = defaultState;
                }

                TrapezoidProfile.State currentState = distProfile.calculate(profileTimer.get());
                ChassisSpeeds profiledChassisSpeeds = new ChassisSpeeds(currentState.velocity, 0, 0);
                DifferentialDriveWheelSpeeds profiledDSpeeds = driveKinematics.toWheelSpeeds(profiledChassisSpeeds);

                // ues P to acquire desired velocity, P to acquire desired position, and feedforwrad to acquire desired velocity
                frontLeftMotor.setVoltage(leftVelPID.calculate(leftEncoder.getVelocity(), profiledDSpeeds.leftMetersPerSecond) + 
                    feedforward.calculate(profiledDSpeeds.leftMetersPerSecond) + 
                    (currentState.position - leftEncoder.getPosition()) * DRIVE_PROFILE_LEFT_POS_P);
                frontRightMotor.setVoltage(rightVelPID.calculate(rightEncoder.getVelocity(), profiledDSpeeds.rightMetersPerSecond) + 
                    feedforward.calculate(profiledDSpeeds.leftMetersPerSecond) + 
                    (currentState.position - rightEncoder.getPosition()) * DRIVE_PROFILE_RIGHT_POS_P);

                if(distProfile.isFinished(profileTimer.get())) {
                    state = defaultState;
                    profileTimer.stop();
                }
            break;
        }

        driveOdometry.update(Rotation2d.fromDegrees(Gyro.getInstance().getRobotAngle()), leftEncoder.getPosition(), rightEncoder.getPosition());

        speedForward = 0;
        speedTurn = 0;
    }

    // Code adapted from WPILib's DifferentialDrive
    private double[] arcadeDrive(double speedForward, double speedTurn) {
        double forward = Math.copySign(speedForward * speedForward, speedForward);
        double turn = Math.copySign(speedTurn * speedTurn, speedTurn);
        double maxInput = Math.copySign(Math.max(Math.abs(forward), Math.abs(turn)), forward);
        
        double speedLeft;
        double speedRight;

        if(forward >= 0.0) {
            if(turn >= 0.0) {
                speedLeft = maxInput;
                speedRight = forward - turn;
            }
            else {
                speedLeft = forward + turn;
                speedRight = maxInput;
            }
        } 
        else {
            if(turn >= 0.0) {
                speedLeft = forward + turn;
                speedRight = maxInput;
            }
            else {
                speedLeft = maxInput;
                speedRight = forward - turn;
            }
        }

        return new double[] {speedLeft, speedRight};
    }

    // speeds should be between -1.0 and 1.0 and they will be squared
    public void manualDrive(double speedForward, double speedTurn) {
        this.speedForward = speedForward;
        this.speedTurn = speedTurn;

        state = defaultState;
    }

    // speedForward should be in m/s
    // speedTurn should be in rad/s
    public void closedLoopDrive(double speedForward, double speedTurn) {
        this.speedForward = speedForward;
        this.speedTurn = speedTurn;

        state = State.CLOSED_LOOP;
    }

    // dist in m
    public void driveDist(double dist) {
        distSetpoint = dist;
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        Gyro.getInstance().zeroRobotAngle();
        distPID.reset();
        state = State.PID_DIST;
    }

    // angle in deg
    public void turnAngle(double angle) {
        angleSetpoint = angle;
        Gyro.getInstance().zeroRobotAngle();
        anglePID.reset();
        state = State.PID_ANGLE;
    }

    // dist in m
    public void driveDistProfile(double dist) {
        distProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(DRIVE_PROFILE_MAX_VEL, DRIVE_PROFILE_MAX_ACC),
            new TrapezoidProfile.State(dist, 0));
        profileTimer.reset();
        profileTimer.start();

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        Gyro.getInstance().zeroRobotAngle();
        distPID.reset();
        state = State.PROFILE_DIST;
    }

    public void driveTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    public void outputValues() {
        SmartDashboard.putNumber("Drive Left Encoder Pos (m)", leftEncoder.getPosition());
        SmartDashboard.putNumber("Drive Right Encoder Pos (m)", rightEncoder.getPosition());
        SmartDashboard.putNumber("Drive Left Encoder Vel (m/s)", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Drive Right Encoder Vel (m/s)", rightEncoder.getVelocity());

        if(distProfile != null) {
            SmartDashboard.putNumber("Drive Profile Time Left", distProfile.timeLeftUntil(profileTimer.get()));
            TrapezoidProfile.State currentState = distProfile.calculate(profileTimer.get());
            SmartDashboard.putNumber("Drive Profile Pos (m)", currentState.position);
            SmartDashboard.putNumber("Drive Profile Vel (m/s)", currentState.velocity);
        }

        SmartDashboard.putNumber("Drive Dist Setpoint", distSetpoint);
        SmartDashboard.putNumber("Drive Angle Setpoint", angleSetpoint);

        SmartDashboard.putString("Drive State", state.name());
    }

    public State getState() {
        return state;
    }
}
