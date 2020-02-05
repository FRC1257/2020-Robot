package frc.robot;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.intake.*;
import frc.robot.commands.indexer.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.elevator.*;
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
    private final Indexer indexer;
    private final Elevator elevator;
    private final Shooter shooter;

    public RobotContainer() {
        driveController = new XboxController(CONTROLLER_DRIVER_ID);
        operatorController = new XboxController(CONTROLLER_OPERATOR_ID);

        intake = new Intake();
        intake.setDefaultCommand(new IntakeNeutralCommand(intake));
        
        indexer = new Indexer();
        indexer.setDefaultCommand(new IndexerNeutralCommand(indexer));

        elevator = new Elevator();
        elevator.setDefaultCommand(new ManualCommand(elevator, operatorController));

        shooter = new Shooter();
        shooter.setDefaultCommand(new ShooterNeutralCommand(shooter));

        configureButtonBindings();
    }

    // defines button -> command mappings
    private void configureButtonBindings() {
        //Intake Bindings
        (new JoystickButton(operatorController, Button.kA.value)).whileHeld(new IntakeEjectCommand(intake));
        (new JoystickButton(operatorController, Button.kB.value)).whileHeld(new IntakeIntakeCommand(intake));

        //Indexer Bindings
        (new JoystickButton(operatorController, Button.kX.value)).whileHeld(new IndexerIntakeCommand(indexer));
        (new JoystickButton(operatorController, Button.kY.value)).whileHeld(new IndexerIntakeCommand(indexer));
        (new JoystickButton(operatorController, Button.kBumperLeft.value)).whileHeld(new IndexerIntakeCommand(indexer));
        (new JoystickButton(operatorController, Button.kBumperRight.value)).whileHeld(new IndexerIntakeCommand(indexer));

        //Shooting Bindings
        (new JoystickButton(operatorController, Button.kA.value)).whileHeld(new ShooterShootingCommand(shooter));
        (new JoystickButton(operatorController, Button.kB.value)).whileHeld(new ShooterPIDCommand(shooter));
    }
}
