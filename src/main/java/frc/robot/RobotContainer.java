package frc.robot;

import static frc.robot.Constants.*; // import the Constants class so we can use the variables in it
                                     // easily

import edu.wpi.first.wpilibj.XboxController;
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
        
    }

    public void outputValues() {
        drivetrain.outputValues();
        Gyro.getInstance().outputValues();
    }
}
