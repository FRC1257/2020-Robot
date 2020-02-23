package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class SnailController extends XboxController {

    public SnailController(int port) {
        super(port);
    }

    public JoystickButton getButton(int id) {
        return new JoystickButton(this, id);
    }

    public XboxTrigger getTrigger(Hand hand) {
        return new XboxTrigger(this, hand);
    }

    public double getLeftY() {
        return -applyDeadband(getY(Hand.kLeft));
    }

    public double getLeftX() {
        return applyDeadband(getX(Hand.kLeft));
    }

    public double getRightY() {
        return -applyDeadband(getY(Hand.kRight));
    }

    public double getRightX() {
        return applyDeadband(getX(Hand.kRight));
    }

    public double getDriveForward() {
        if (getAButton()) {
            return getLeftY();
        } else if (getBumper(Hand.kRight)) {
            return getRightY();
        } else if (getBumper(Hand.kLeft)) {
            return getLeftY();
        }
        return 0;
    }

    public double getDriveTurn() {
        if (getAButton()) {
            return getLeftX();
        } else if (getBumper(Hand.kRight)) {
            return getLeftX();
        } else if (getBumper(Hand.kLeft)) {
            return getRightX();
        }
        return 0;
    }

    public static double applyDeadband(double value) {
        if (Math.abs(value) < 0.08) return 0;
        else return value;
    }
}