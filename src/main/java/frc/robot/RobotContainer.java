package frc.robot;

import static frc.robot.Constants.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.indexer.*;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

    private final XboxController driveController;
    private final XboxController operatorController;
    private final Intake intake;
    private final Intake indexer;


    public RobotContainer() {
        driveController = new XboxController(CONTROLLER_DRIVE_ID);
        operatorController = new XboxController(CONTROLLER_OPERATOR_ID);

        intake = new Intake();
        intake.setDefaultCommand(new IntakeNeutralCommand(intake));
        
        


        configureButtonBindings();
    }

    // defines button -> command mappings
    private void configureButtonBindings() {

    }
}