package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    Servo servo;

    XboxController controller;

    @Override
    public void robotInit() {
        servo = new Servo(0);

        controller = new XboxController(0);
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Servo Position", servo.getPosition());
        SmartDashboard.putNumber("Servo Speed", servo.getSpeed());
        SmartDashboard.putNumber("Servo Angle", servo.getAngle());

        if(controller.getAButton()) {

        }
        servo.set(controller.getY(Hand.kLeft));
    }
}
