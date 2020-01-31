package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.ReverseDriveCommand;
import frc.robot.commands.drivetrain.ManualDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Gyro;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final Drivetrain drivetrain;

    private final XboxController controller;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        controller = new XboxController(CONTROLLER_ID);

        drivetrain = new Drivetrain();
        drivetrain.setDefaultCommand(new ManualDriveCommand(drivetrain, controller));
        
        configureButtonBindings();
    }

    /**
     * Use this method to define your button -> command mappings.
     */
    private void configureButtonBindings() {
        (new JoystickButton(controller, XboxController.Button.kY.value)).whenPressed(new ReverseDriveCommand(drivetrain);
    }

    public void outputValues() {
        drivetrain.outputValues();
        Gyro.getInstance().outputValues();
    }

    public Command getAutoCommand() {
        return null;
    }
}
