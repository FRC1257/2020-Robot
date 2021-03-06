package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.util.Gyro;

import static frc.robot.Constants.ElectricalLayout;
import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.NEO_CURRENT_LIMIT;

public class Drivetrain extends SnailSubsystem {
  
    private final CANSparkMax frontLeftMotor;
    private final CANSparkMax frontRightMotor;
    private final CANSparkMax backLeftMotor;
    private final CANSparkMax backRightMotor;

    private final CANEncoder leftEncoder;
    private final CANEncoder rightEncoder;

    private final CANPIDController leftPID;
    private final CANPIDController rightPID;

    private final DifferentialDriveKinematics driveKinematics;
    private final DifferentialDriveOdometry driveOdometry;

    private final PIDController distPID;
    private final PIDController anglePID;
    
    private double distSetpoint;
    private double angleSetpoint;

    private TrapezoidProfile distProfile;
    private final Timer pathTimer;

    private final RamseteController ramseteController;
    private Trajectory trajectory;

    public enum State {
        MANUAL,
        CLOSED_LOOP,
        PID_DIST,
        PID_ANGLE,
        PROFILE_DIST,
        RAMSETE
    }
    private final State defaultState = State.MANUAL;
    private State state = defaultState;

    private double speedForward;
    private double speedTurn;
    private boolean reversed;
    private boolean slowTurning;

    public Drivetrain() {
        frontLeftMotor = new CANSparkMax(ElectricalLayout.DRIVE_FRONT_LEFT, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(ElectricalLayout.DRIVE_FRONT_RIGHT, MotorType.kBrushless);
        backLeftMotor = new CANSparkMax(ElectricalLayout.DRIVE_BACK_LEFT, MotorType.kBrushless);
        backRightMotor = new CANSparkMax(ElectricalLayout.DRIVE_BACK_RIGHT, MotorType.kBrushless);

        frontLeftMotor.restoreFactoryDefaults();
        frontRightMotor.restoreFactoryDefaults();
        backLeftMotor.restoreFactoryDefaults();
        backRightMotor.restoreFactoryDefaults();

        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        frontRightMotor.setIdleMode(IdleMode.kBrake);
        backLeftMotor.setIdleMode(IdleMode.kCoast);
        backRightMotor.setIdleMode(IdleMode.kCoast);

        frontLeftMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        frontRightMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        backLeftMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        backRightMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        frontRightMotor.setInverted(true);

        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);

        leftEncoder = new CANEncoder(frontLeftMotor);
        rightEncoder = new CANEncoder(frontRightMotor);
        leftEncoder.setPositionConversionFactor(Math.PI * Constants.Drivetrain.DRIVE_WHEEL_DIAM_M / Constants.Drivetrain.DRIVE_GEARBOX_REDUCTION);
        rightEncoder.setPositionConversionFactor(Math.PI * Constants.Drivetrain.DRIVE_WHEEL_DIAM_M / Constants.Drivetrain.DRIVE_GEARBOX_REDUCTION);
        leftEncoder.setVelocityConversionFactor(Math.PI * Constants.Drivetrain.DRIVE_WHEEL_DIAM_M / 60.0 / Constants.Drivetrain.DRIVE_GEARBOX_REDUCTION);
        rightEncoder.setVelocityConversionFactor(Math.PI * Constants.Drivetrain.DRIVE_WHEEL_DIAM_M / 60.0 / Constants.Drivetrain.DRIVE_GEARBOX_REDUCTION);
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        leftPID = new CANPIDController(frontLeftMotor);
        rightPID = new CANPIDController(frontRightMotor);
        leftPID.setP(Constants.Drivetrain.DRIVE_LEFT_VEL_PID_P, Constants.Drivetrain.DRIVE_PID_SLOT_VEL);
        leftPID.setFF(Constants.Drivetrain.DRIVE_LEFT_VEL_PID_F, Constants.Drivetrain.DRIVE_PID_SLOT_VEL);
        rightPID.setP(Constants.Drivetrain.DRIVE_RIGHT_VEL_PID_P, Constants.Drivetrain.DRIVE_PID_SLOT_VEL);
        rightPID.setFF(Constants.Drivetrain.DRIVE_RIGHT_VEL_PID_F, Constants.Drivetrain.DRIVE_PID_SLOT_VEL);

        driveKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.DRIVE_TRACK_WIDTH_M);
        driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(Gyro.getInstance().getRobotAngle()));

        distPID = new PIDController(Constants.Drivetrain.DRIVE_DIST_PID[0], Constants.Drivetrain.DRIVE_DIST_PID[1], Constants.Drivetrain.DRIVE_DIST_PID[2]);
        distPID.setTolerance(Constants.Drivetrain.DRIVE_DIST_PID_TOLERANCE);
        
        anglePID = new PIDController(Constants.Drivetrain.DRIVE_ANGLE_PID[0], Constants.Drivetrain.DRIVE_ANGLE_PID[1], Constants.Drivetrain.DRIVE_ANGLE_PID[2]);
        anglePID.setTolerance(Constants.Drivetrain.DRIVE_ANGLE_PID_TOLERANCE);
        anglePID.enableContinuousInput(-180.0, 180.0);

        ramseteController = new RamseteController(Constants.Drivetrain.DRIVE_RAMSETE_B, Constants.Drivetrain.DRIVE_RAMSETE_ZETA);

        pathTimer = new Timer();

        reset();
    }

    public void reset() {
        state = defaultState;
        distSetpoint = -1.257;
        angleSetpoint = -1.257;
        pathTimer.stop();
        pathTimer.reset();
        reversed = false;
        slowTurning = false;
    }

    @Override
    public void update() {
        switch(state) {
            case MANUAL:
                double[] arcadeSpeeds = arcadeDrive(reversed ? -speedForward : speedForward,
                    slowTurning ? speedTurn * Constants.Drivetrain.DRIVE_REDUCE_TURNING_CONSTANT : speedTurn);

                frontLeftMotor.set(arcadeSpeeds[0]);
                frontRightMotor.set(arcadeSpeeds[1]);
                break;
            case CLOSED_LOOP:
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(reversed ? -speedForward : speedForward, 0,
                    slowTurning ? speedTurn * Constants.Drivetrain.DRIVE_REDUCE_TURNING_CONSTANT : speedTurn);
                DifferentialDriveWheelSpeeds dSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);

                leftPID.setReference(dSpeeds.leftMetersPerSecond, ControlType.kVelocity, Constants.Drivetrain.DRIVE_PID_SLOT_VEL);
                rightPID.setReference(dSpeeds.rightMetersPerSecond, ControlType.kVelocity, Constants.Drivetrain.DRIVE_PID_SLOT_VEL);
                break;
            case PID_DIST:
                if (distSetpoint == -1.257) {
                    state = defaultState;
                    break;
                }

                double forward = distPID.calculate(leftEncoder.getPosition(), distSetpoint);
                forward = Math.min(forward, Constants.Drivetrain.DRIVE_DIST_PID_MAX_OUTPUT);
                forward = Math.max(forward, -Constants.Drivetrain.DRIVE_DIST_PID_MAX_OUTPUT);
                // Apply PID controller to forward speed and basic P control to angle
                double[] pidDistArcadeSpeeds = arcadeDrive(forward,
                    -Gyro.getInstance().getRobotAngle() * Constants.Drivetrain.DRIVE_MAINTAIN_ANGLE_PID_P);

                frontLeftMotor.set(pidDistArcadeSpeeds[0]);
                frontRightMotor.set(pidDistArcadeSpeeds[1]);

                // if (distPID.atSetpoint()) {
                //     state = defaultState;
                //     distSetpoint = -1.257;
                // }
                break;
            case PID_ANGLE:
                if (angleSetpoint == -1.257) {
                    state = defaultState;
                    break;
                }

                double output = anglePID.calculate(Gyro.getInstance().getRobotAngle(), angleSetpoint);
                output = Math.min(output, Constants.Drivetrain.DRIVE_ANGLE_PID_MAX_OUTPUT);
                output = Math.max(output, -Constants.Drivetrain.DRIVE_ANGLE_PID_MAX_OUTPUT);
                if (output > 0.5) output = 0.5;
                if (output < -0.5) output = -0.5;

                // Apply PID controller to forward speed and basic P control to angle
                double[] pidAngleArcadeSpeeds = arcadeDrive(0, output);

                frontLeftMotor.set(pidAngleArcadeSpeeds[0]);
                frontRightMotor.set(pidAngleArcadeSpeeds[1]);

                // if (anglePID.atSetpoint()) {
                //     state = defaultState;
                //     angleSetpoint = -1.257;
                // }
                break;
            case PROFILE_DIST:
                if (distProfile == null) {
                    state = defaultState;
                    break;
                }

                TrapezoidProfile.State currentStateProf = distProfile.calculate(pathTimer.get());
                ChassisSpeeds profiledChassisSpeeds = new ChassisSpeeds(currentStateProf.velocity, 0, 0);
                DifferentialDriveWheelSpeeds profiledDSpeeds = driveKinematics.toWheelSpeeds(profiledChassisSpeeds);

                double positionError = currentStateProf.position - leftEncoder.getPosition();

                // use P and feedforward to acquire desired velocity, P to acquire desired position
                leftPID.setReference(profiledDSpeeds.leftMetersPerSecond, ControlType.kVelocity, Constants.Drivetrain.DRIVE_PID_SLOT_VEL,
                    positionError * Constants.Drivetrain.DRIVE_PROFILE_LEFT_POS_P, ArbFFUnits.kPercentOut);
                    rightPID.setReference(profiledDSpeeds.rightMetersPerSecond, ControlType.kVelocity, Constants.Drivetrain.DRIVE_PID_SLOT_VEL,
                    positionError * Constants.Drivetrain.DRIVE_PROFILE_RIGHT_POS_P, ArbFFUnits.kPercentOut);

                if (distProfile.isFinished(pathTimer.get())) {
                    state = defaultState;
                    pathTimer.stop();
                    distProfile = null;
                }
                break;
            case RAMSETE:
                if (trajectory == null) {
                    state = defaultState;
                    break;
                }

                Trajectory.State currentStateTraj = trajectory.sample(pathTimer.get());
                ChassisSpeeds ramseteChassisSpeeds = ramseteController.calculate(driveOdometry.getPoseMeters(), currentStateTraj);
                DifferentialDriveWheelSpeeds ramseteDSpeeds = driveKinematics.toWheelSpeeds(ramseteChassisSpeeds);

                // use P and feedforward to acquire desired velocity
                leftPID.setReference(ramseteDSpeeds.leftMetersPerSecond, ControlType.kVelocity, Constants.Drivetrain.DRIVE_PID_SLOT_VEL);
                rightPID.setReference(ramseteDSpeeds.rightMetersPerSecond, ControlType.kVelocity, Constants.Drivetrain.DRIVE_PID_SLOT_VEL);

                if (trajectory.getTotalTimeSeconds() <= pathTimer.get()) {
                    state = defaultState;
                    pathTimer.stop();
                    trajectory = null;
                }
                break;
        }

        // negative angle so that counter clockwise = positive angle
        driveOdometry.update(Rotation2d.fromDegrees(-Gyro.getInstance().getRobotAngle()), leftEncoder.getPosition(), rightEncoder.getPosition());

        speedForward = 0;
        speedTurn = 0;
    }

    // Code adapted from WPILib's DifferentialDrive
    private double[] arcadeDrive(double speedForward, double speedTurn) {
        double forward = Math.copySign(speedForward * speedForward, speedForward);
        double turn = Math.copySign(speedTurn * speedTurn, speedTurn);

        if (Math.abs(forward) < 0.02) forward = 0.0;
        if (Math.abs(turn) < 0.02) turn = 0.0;

        double maxInput = Math.copySign(Math.max(Math.abs(forward), Math.abs(turn)), forward);
        
        double speedLeft;
        double speedRight;

        if (forward >= 0.0) {
            if (turn >= 0.0) {
                speedLeft = maxInput;
                speedRight = forward - turn;
            }
            else {
                speedLeft = forward + turn;
                speedRight = maxInput;
            }
        } 
        else {
            if (turn >= 0.0) {
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
        Gyro.getInstance().zeroRobotAngle();
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        distSetpoint = dist;
        distPID.reset();
        state = State.PID_DIST;
    }

    // angle in deg
    public void turnAngle(double angle) {
        Gyro.getInstance().zeroRobotAngle();
        angleSetpoint = angle;
        anglePID.reset();
        anglePID.setSetpoint(angle);
        pathTimer.stop();
        pathTimer.reset();
        state = State.PID_ANGLE;
    }

    // dist in m
    public void driveDistProfile(double dist) {
        Gyro.getInstance().zeroRobotAngle();
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        distProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(Constants.Drivetrain.DRIVE_PATH_MAX_VEL, Constants.Drivetrain.DRIVE_PATH_MAX_ACC),
            new TrapezoidProfile.State(dist, 0),
            new TrapezoidProfile.State(0, 0));
        pathTimer.reset();
        pathTimer.start();
        state = State.PROFILE_DIST;
    }

    // drives a certain trajectory
    public void driveTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
        pathTimer.reset();
        pathTimer.start();
        state = State.RAMSETE;
    }

    // ends any PID commands
    public void endPID() {
        angleSetpoint = -1.257;
        distSetpoint = -1.257;
        distProfile = null;
        state = defaultState;
    }

    // toggles reverse drive
    public void toggleReverse() {
        reversed = !reversed;
    }

    // toggles turning slowdown
    public void toggleTurnSlowdown() {
        slowTurning = !slowTurning;
    }

    // sets the robot pose to a given position
    // should only be called once at the beginning of autonomous
    public void setRobotPose(Pose2d pose) {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        
        driveOdometry.resetPosition(pose, new Rotation2d(-Gyro.getInstance().getRobotAngle()));
    }

    @Override
    public void outputValues() {
        SmartDashboard.putBooleanArray("Drive Toggles", new boolean[] {reversed, slowTurning});

        if(SmartDashboard.getBoolean("Testing", false)) {
            if (distProfile != null) {
                SmartDashboard.putNumber("Drive Profile Time Left", distProfile.timeLeftUntil(pathTimer.get()));
                TrapezoidProfile.State currentState = distProfile.calculate(pathTimer.get());
                SmartDashboard.putNumber("Drive Profile Pos (m)", currentState.position);
                SmartDashboard.putNumber("Drive Profile Vel (mps)", currentState.velocity);
            }
            if (trajectory != null) {
                SmartDashboard.putNumber("Trajectory Time Left", trajectory.getTotalTimeSeconds() - pathTimer.get());
            }

            SmartDashboard.putNumberArray("Drive Encoders (Lp, Rp, Lv, Rv)", new double[] {
                leftEncoder.getPosition(), rightEncoder.getPosition(),
                leftEncoder.getVelocity(), rightEncoder.getVelocity()
            });

            // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(reversed ? -speedForward : speedForward, 0,
            // slowTurning ? speedTurn * DRIVE_REDUCE_TURNING_CONSTANT : speedTurn);
            // DifferentialDriveWheelSpeeds dSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);
            // SmartDashboard.putNumberArray("Closed Loop Vel", new double[] {
            //     leftEncoder.getVelocity(), rightEncoder.getVelocity(),
            //     dSpeeds.leftMetersPerSecond, dSpeeds.rightMetersPerSecond
            // });
            // SmartDashboard.putNumberArray("Drive PID Dist", new double[] {
            //     leftEncoder.getPosition(),
            //     distSetpoint
            // });
            SmartDashboard.putNumberArray("Drive PID Angle", new double[] {
                    Gyro.getInstance().getRobotAngle(),
                    angleSetpoint
            });

            SmartDashboard.putString("Drive State", state.name());
        }
    }

    @Override
    public void setUpConstantTuning() {
        SmartDashboard.putNumber("Drive Reduce Turning Constant", DRIVE_REDUCE_TURNING_CONSTANT);

        SmartDashboard.putNumber("Drive PID Vel Left kP", DRIVE_LEFT_VEL_PID_P);
        SmartDashboard.putNumber("Drive PID Vel Left kFF", DRIVE_LEFT_VEL_PID_F);
        SmartDashboard.putNumber("Drive PID Vel Right kP", DRIVE_RIGHT_VEL_PID_P);
        SmartDashboard.putNumber("Drive PID Vel Right kFF", DRIVE_LEFT_VEL_PID_F);

        SmartDashboard.putNumber("Drive PID Dist kP", DRIVE_DIST_PID[0]);
        SmartDashboard.putNumber("Drive PID Dist kI", DRIVE_DIST_PID[1]);
        SmartDashboard.putNumber("Drive PID Dist kD", DRIVE_DIST_PID[2]);
        SmartDashboard.putNumber("Drive PID Maintain Angle kP", DRIVE_MAINTAIN_ANGLE_PID_P);
        
        SmartDashboard.putNumber("Drive PID Angle kP", DRIVE_ANGLE_PID[0]);
        SmartDashboard.putNumber("Drive PID Angle kI", DRIVE_ANGLE_PID[1]);
        SmartDashboard.putNumber("Drive PID Angle kD", DRIVE_ANGLE_PID[2]);

        SmartDashboard.putNumber("Drive Profile Pos Left kP", DRIVE_PROFILE_LEFT_POS_P);
        SmartDashboard.putNumber("Drive Profile Pos Right kP", DRIVE_PROFILE_RIGHT_POS_P);
    }

    @Override
    public void getConstantTuning() {
        Constants.Drivetrain.DRIVE_REDUCE_TURNING_CONSTANT = SmartDashboard.getNumber("Drive Reduce Turning Constant",
                Constants.Drivetrain.DRIVE_REDUCE_TURNING_CONSTANT);

        DRIVE_LEFT_VEL_PID_P = SmartDashboard.getNumber("Drive PID Vel Left kP", DRIVE_LEFT_VEL_PID_P);
        DRIVE_LEFT_VEL_PID_F = SmartDashboard.getNumber("Drive PID Vel Left kFF", DRIVE_LEFT_VEL_PID_F);
        DRIVE_RIGHT_VEL_PID_P = SmartDashboard.getNumber("Drive PID Vel Right kP", DRIVE_RIGHT_VEL_PID_P);
        DRIVE_RIGHT_VEL_PID_F = SmartDashboard.getNumber("Drive PID Vel Right kFF", DRIVE_LEFT_VEL_PID_F);

        DRIVE_DIST_PID[0] = SmartDashboard.getNumber("Drive PID Dist kP", DRIVE_DIST_PID[0]);
        DRIVE_DIST_PID[1] = SmartDashboard.getNumber("Drive PID Dist kI", DRIVE_DIST_PID[1]);
        DRIVE_DIST_PID[2] = SmartDashboard.getNumber("Drive PID Dist kD", DRIVE_DIST_PID[2]);
        DRIVE_MAINTAIN_ANGLE_PID_P = SmartDashboard.getNumber("Drive PID Maintain Angle kP", DRIVE_MAINTAIN_ANGLE_PID_P);

        DRIVE_ANGLE_PID[0] = SmartDashboard.getNumber("Drive PID Angle kP", DRIVE_ANGLE_PID[0]);
        DRIVE_ANGLE_PID[1] = SmartDashboard.getNumber("Drive PID Angle kI", DRIVE_ANGLE_PID[1]);
        DRIVE_ANGLE_PID[2] = SmartDashboard.getNumber("Drive PID Angle kD", DRIVE_ANGLE_PID[2]);

        DRIVE_PROFILE_LEFT_POS_P = SmartDashboard.getNumber("Drive Profile Pos Left kP", DRIVE_PROFILE_LEFT_POS_P);
        DRIVE_PROFILE_RIGHT_POS_P = SmartDashboard.getNumber("Drive Profile Pos Right kP", DRIVE_PROFILE_RIGHT_POS_P);

        if (leftPID.getP() != DRIVE_LEFT_VEL_PID_P) {
            leftPID.setP(DRIVE_LEFT_VEL_PID_P);
        }
        if (leftPID.getFF() != DRIVE_LEFT_VEL_PID_F) {
            leftPID.setFF(DRIVE_LEFT_VEL_PID_F);
        }
        if (rightPID.getP() != DRIVE_RIGHT_VEL_PID_P) {
            rightPID.setP(DRIVE_RIGHT_VEL_PID_P);
        }
        if (rightPID.getFF() != DRIVE_RIGHT_VEL_PID_F) {
            rightPID.setFF(DRIVE_RIGHT_VEL_PID_F);
        }

        if (distPID.getP() != Constants.Drivetrain.DRIVE_DIST_PID[0]) {
            distPID.setP(Constants.Drivetrain.DRIVE_DIST_PID[0]);
        }
        if (distPID.getI() != Constants.Drivetrain.DRIVE_DIST_PID[1]) {
            distPID.setI(Constants.Drivetrain.DRIVE_DIST_PID[1]);
        }
        if (distPID.getD() != Constants.Drivetrain.DRIVE_DIST_PID[2]) {
            distPID.setD(Constants.Drivetrain.DRIVE_DIST_PID[2]);
        }

        if (anglePID.getP() != Constants.Drivetrain.DRIVE_ANGLE_PID[0]) {
            anglePID.setP(Constants.Drivetrain.DRIVE_ANGLE_PID[0]);
        }
        if (anglePID.getI() != Constants.Drivetrain.DRIVE_ANGLE_PID[1]) {
            anglePID.setI(Constants.Drivetrain.DRIVE_ANGLE_PID[1]);
        }
        if (anglePID.getD() != Constants.Drivetrain.DRIVE_ANGLE_PID[2]) {
            anglePID.setD(Constants.Drivetrain.DRIVE_ANGLE_PID[2]);
        }
    }

    public State getState() {
        return state;
    }
}
