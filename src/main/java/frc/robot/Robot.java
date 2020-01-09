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

        if(controller.getAButton()) {
            pidController1.setReference(SmartDashboard.getNumber("Setpoint", 0), ControlType.kVelocity);
        }
        else if(controller.getBumper(Hand.kLeft)) {
            double val = controller.getY(Hand.kLeft);
            if(Math.abs(val) < 0.05) val = 0;
            motor1.set(val);
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
