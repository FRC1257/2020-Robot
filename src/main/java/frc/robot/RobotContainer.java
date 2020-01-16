package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.elevator.ManualCommand;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

    private final XboxController driveController;
    private final XboxController operatorController;

    private final Elevator elevator;

    public RobotContainer() {
        driveController = new XboxController(CONTROLLER_DRIVE_ID);
        operatorController = new XboxController(CONTROLLER_OPERATOR_ID);

        elevator = new Elevator();
        elevator.setDefaultCommand(new ManualCommand(elevator, operatorController));

        configureButtonBindings();
    }

    // defines button -> command mappings
    private void configureButtonBindings() {

    }
}