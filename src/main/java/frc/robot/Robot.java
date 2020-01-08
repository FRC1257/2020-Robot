package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    CANSparkMax motor1;
    CANEncoder encoder1;
    CANPIDController pidController1;

    XboxController controller;

    @Override
    public void teleopInit() {
        motor1 = new CANSparkMax(2, MotorType.kBrushless);
        motor1.restoreFactoryDefaults();
        motor1.setIdleMode(IdleMode.kCoast);
        encoder1 = motor1.getEncoder();
        pidController1 = motor1.getPIDController();

        pidController1.setP(0.0);
        pidController1.setI(0.0);
        pidController1.setD(0.0);
        pidController1.setFF(0.0);

        SmartDashboard.putNumber("kP", 0.0);
        SmartDashboard.putNumber("kI", 0.0);
        SmartDashboard.putNumber("kD", 0.0);
        SmartDashboard.putNumber("kFF", 0.000160);
        SmartDashboard.putNumber("Setpoint", 4500);

        controller = new XboxController(0);
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Encoder 1 Pos", encoder1.getPosition());
        SmartDashboard.putNumber("Encoder 1 Vel", encoder1.getVelocity());

        if(controller.getBumper(Hand.kLeft)) {
            double val = controller.getY(Hand.kLeft);
            if(Math.abs(val) < 0.05) val = 0;
            motor1.set(val);
        }

        if(controller.getAButton()) {
            pidController1.setReference(SmartDashboard.getNumber("Setpoint", 0), ControlType.kVelocity);
        }
        else {
            motor1.set(0);
        }

        if(pidController1.getP() != SmartDashboard.getNumber("kP", 0)) {
            pidController1.setP(SmartDashboard.getNumber("kP", 0));
        }
        if(pidController1.getI() != SmartDashboard.getNumber("kI", 0)) {
            pidController1.setI(SmartDashboard.getNumber("kI", 0));
        }
        if(pidController1.getD() != SmartDashboard.getNumber("kD", 0)) {
            pidController1.setD(SmartDashboard.getNumber("kD", 0));
        }
        if(pidController1.getFF() != SmartDashboard.getNumber("kFF", 0)) {
            pidController1.setFF(SmartDashboard.getNumber("kFF", 0));
        }
    }
}

// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANPIDController;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.ControlType;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// /**
//  * This is a demo program showing the use of the RobotDrive class, specifically
//  * it contains the code necessary to operate a robot with tank drive.
//  */
// public class Robot extends TimedRobot {
//   private XboxController m_stick;
//   private static final int deviceID = 2;
//   private CANSparkMax m_motor;
//   private CANPIDController m_pidController;
//   private CANEncoder m_encoder;
//   public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

//   @Override
//   public void robotInit() {
//     m_stick = new XboxController(0);

//     // initialize motor
//     m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

//     /**
//      * The RestoreFactoryDefaults method can be used to reset the configuration parameters
//      * in the SPARK MAX to their factory default state. If no argument is passed, these
//      * parameters will not persist between power cycles
//      */
//     m_motor.restoreFactoryDefaults();

//     /**
//      * In order to use PID functionality for a controller, a CANPIDController object
//      * is constructed by calling the getPIDController() method on an existing
//      * CANSparkMax object
//      */
//     m_pidController = m_motor.getPIDController();

//     // Encoder object created to display position values
//     m_encoder = m_motor.getEncoder();

//     // PID coefficients
//     kP = 5e-5; 
//     kI = 1e-6;
//     kD = 0; 
//     kIz = 0; 
//     kFF = 0; 
//     kMaxOutput = 1; 
//     kMinOutput = -1;
//     maxRPM = 5700;

//     // set PID coefficients
//     m_pidController.setP(kP);
//     m_pidController.setI(kI);
//     m_pidController.setD(kD);
//     m_pidController.setIZone(kIz);
//     m_pidController.setFF(kFF);
//     m_pidController.setOutputRange(kMinOutput, kMaxOutput);

//     // display PID coefficients on SmartDashboard
//     SmartDashboard.putNumber("P Gain", kP);
//     SmartDashboard.putNumber("I Gain", kI);
//     SmartDashboard.putNumber("D Gain", kD);
//     SmartDashboard.putNumber("I Zone", kIz);
//     SmartDashboard.putNumber("Feed Forward", kFF);
//     SmartDashboard.putNumber("Max Output", kMaxOutput);
//     SmartDashboard.putNumber("Min Output", kMinOutput);
//   }

//   @Override
//   public void teleopPeriodic() {
//     // read PID coefficients from SmartDashboard
//     double p = SmartDashboard.getNumber("P Gain", 0);
//     double i = SmartDashboard.getNumber("I Gain", 0);
//     double d = SmartDashboard.getNumber("D Gain", 0);
//     double iz = SmartDashboard.getNumber("I Zone", 0);
//     double ff = SmartDashboard.getNumber("Feed Forward", 0);
//     double max = SmartDashboard.getNumber("Max Output", 0);
//     double min = SmartDashboard.getNumber("Min Output", 0);

//     // if PID coefficients on SmartDashboard have changed, write new values to controller
//     if((p != kP)) { m_pidController.setP(p); kP = p; }
//     if((i != kI)) { m_pidController.setI(i); kI = i; }
//     if((d != kD)) { m_pidController.setD(d); kD = d; }
//     if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
//     if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
//     if((max != kMaxOutput) || (min != kMinOutput)) { 
//       m_pidController.setOutputRange(min, max); 
//       kMinOutput = min; kMaxOutput = max; 
//     }

//     /**
//      * PIDController objects are commanded to a set point using the 
//      * SetReference() method.
//      * 
//      * The first parameter is the value of the set point, whose units vary
//      * depending on the control type set in the second parameter.
//      * 
//      * The second parameter is the control type can be set to one of four 
//      * parameters:
//      *  com.revrobotics.ControlType.kDutyCycle
//      *  com.revrobotics.ControlType.kPosition
//      *  com.revrobotics.ControlType.kVelocity
//      *  com.revrobotics.ControlType.kVoltage
//      */
//     double setPoint = m_stick.getY(Hand.kLeft)*maxRPM;
//     m_pidController.setReference(setPoint, ControlType.kVelocity);
    
//     SmartDashboard.putNumber("SetPoint", setPoint);
//     SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
//   }
// }