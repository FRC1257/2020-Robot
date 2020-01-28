package frc.robot;

import static frc.robot.Constants.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.indexer.*;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

    private final XboxController operatorController;
    private final Intake intake;
    private final Indexer indexer;


    public RobotContainer() {
        operatorController = new XboxController(CONTROLLER_OPERATOR_ID);

        intake = new Intake();
        intake.setDefaultCommand(new IntakeNeutralCommand(intake));
        
        indexer = new Indexer();
        indexer.setDefaultCommand(new IndexerNeutralCommand(indexer));

        configureButtonBindings();
    }

    // defines button -> command mappings
    private void configureButtonBindings() {
        //Intake Bindings
        (new JoystickButton(operatorController, Button.kA.value)).whileHeld(new IntakeEjectCommand(intake));
        (new JoystickButton(operatorController, Button.kB.value)).whileHeld(new IntakeIntakeCommand(intake));

        (new JoystickButton(operatorController, Button.kX.value)).whileHeld(new IndexerIntakeCommand(indexer));
        (new JoystickButton(operatorController, Button.kY.value)).whileHeld(new IndexerIntakeCommand(indexer));
        (new JoystickButton(operatorController, Button.kBumperLeft.value)).whileHeld(new IndexerIntakeCommand(indexer));
        (new JoystickButton(operatorController, Button.kBumperRight.value)).whileHeld(new IndexerIntakeCommand(indexer));
    }
}
