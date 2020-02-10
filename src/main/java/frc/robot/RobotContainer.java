package frc.robot;

import static frc.robot.Constants.*;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.drivetrain.ReverseDriveCommand;
import frc.robot.commands.drivetrain.SlowTurnCommand;
import frc.robot.commands.auto.BottomAuto;
import frc.robot.commands.auto.DriveBaselineAuto;
import frc.robot.commands.auto.MiddleGenBottomAuto;
import frc.robot.commands.auto.MiddleGenTopAuto;
import frc.robot.commands.auto.MiddleTrenchAuto;
import frc.robot.commands.auto.TopGenBottomAuto;
import frc.robot.commands.auto.TopGenTopAuto;
import frc.robot.commands.auto.TopTrenchAuto;
import frc.robot.commands.drivetrain.ManualDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Gyro;
import frc.robot.util.XboxTrigger;
import frc.robot.commands.intake.*;
import frc.robot.commands.indexer.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.elevator.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final XboxController driveController;
    private final XboxController operatorController;
    
    private ArrayList<SnailSubsystem> subsystems;
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Indexer indexer;
    private final Elevator elevator;
    private final Shooter shooter;

    private SendableChooser<Constants.AutoPosition> autoPositionChooser;
    private SendableChooser<Constants.AutoGoal> autoGoalChooser;

    private int outputCounter;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
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
        
        drivetrain = new Drivetrain();
        drivetrain.setDefaultCommand(new ManualDriveCommand(drivetrain, driveController));
        
        subsystems.add(intake);
        subsystems.add(indexer);
        subsystems.add(elevator);
        subsystems.add(shooter);
        subsystems.add(drivetrain);

        configureAutoChoosers();
        configureButtonBindings();
        outputCounter = 0;
    }

    /**
     * Use this method to define your button -> command mappings.
     */
    private void configureButtonBindings() {
        // Drivetrain Bindings
        (new JoystickButton(driveController, XboxController.Button.kY.value)).whenPressed(new ReverseDriveCommand(drivetrain));
        (new JoystickButton(driveController, XboxController.Button.kStart.value)).whenPressed(new SlowTurnCommand(drivetrain));

        // Intake Bindings
        (new JoystickButton(operatorController, Button.kA.value)).whileHeld(new IntakeEjectCommand(intake));
        (new JoystickButton(operatorController, Button.kB.value)).whileHeld(new IntakeIntakeCommand(intake));

        // Indexer Bindings
        (new JoystickButton(operatorController, Button.kX.value)).whileHeld(new IndexerIntakeCommand(indexer));
        (new JoystickButton(operatorController, Button.kY.value)).whenPressed(new IndexerPIDCommand(indexer));
        (new JoystickButton(operatorController, Button.kBumperLeft.value)).whileHeld(new IndexerEjectCommand(indexer));
        (new JoystickButton(operatorController, Button.kBumperRight.value)).whileHeld(new IndexerShootCommand(indexer));

        // Elevator Bindings
        (new JoystickButton(operatorController, Button.kX.value)).whileHeld(new PIDCommand(elevator));
        (new JoystickButton(operatorController, Button.kY.value)).whenPressed(new ToggleLockCommand(elevator));

        // Shooting Bindings
        (new XboxTrigger(operatorController, Hand.kLeft)).whileActiveOnce(new ShooterShootingCommand(shooter));
        (new XboxTrigger(operatorController, Hand.kRight)).whileActiveOnce(new ShooterPIDCommand(shooter));
    }

    public Command getAutoCommand() {
        Constants.AutoPosition position = autoPositionChooser.getSelected();
        Constants.AutoGoal goal = autoGoalChooser.getSelected();

        if (position == Constants.AutoPosition.BOTTOM) return new BottomAuto(drivetrain, indexer, shooter);

        switch (goal) {
            case TRENCH:
                if (position == Constants.AutoPosition.TOP) return new TopTrenchAuto(drivetrain, indexer, shooter);
                else if (position == Constants.AutoPosition.MIDDLE) return new MiddleTrenchAuto(drivetrain, indexer, shooter);
            case GEN_TOP: 
                if (position == Constants.AutoPosition.TOP) return new TopGenTopAuto(drivetrain, indexer, shooter);
                else if (position == Constants.AutoPosition.MIDDLE) return new MiddleGenTopAuto(drivetrain, indexer, shooter);
            case GEN_BOTTOM:
                if (position == Constants.AutoPosition.TOP) return new TopGenBottomAuto(drivetrain, indexer, shooter);
                else if (position == Constants.AutoPosition.MIDDLE) return new MiddleGenBottomAuto(drivetrain, indexer, shooter);
            case DEFAULT:
            default:
                return new DriveBaselineAuto(drivetrain);
        }
    }

    public void configureAutoChoosers() {
        autoPositionChooser = new SendableChooser<Constants.AutoPosition>();
        autoGoalChooser = new SendableChooser<Constants.AutoGoal>();

        autoPositionChooser.setDefaultOption("Top Start", Constants.AutoPosition.TOP);
        autoPositionChooser.addOption("Middle Start", Constants.AutoPosition.MIDDLE);
        autoPositionChooser.addOption("Bottom Start", Constants.AutoPosition.BOTTOM);

        autoGoalChooser.setDefaultOption("Default Drive", Constants.AutoGoal.DEFAULT);
        autoGoalChooser.addOption("Trench", Constants.AutoGoal.TRENCH);
        autoGoalChooser.addOption("Generator Top", Constants.AutoGoal.GEN_TOP);
        autoGoalChooser.addOption("Generator Bottom", Constants.AutoGoal.GEN_BOTTOM);

        SmartDashboard.putData(autoPositionChooser);
        SmartDashboard.putData(autoGoalChooser);
    }

    public void outputValues() {
        subsystems.get(outputCounter).outputValues();

        if (outputCounter == subsystems.size()) {
            Gyro.getInstance().outputValues();
        }

        outputCounter = (outputCounter + 1) % (subsystems.size() + 1);
    }

    public void setConstantTuning() {
        subsystems.forEach((s) -> s.setConstantTuning());
    }

    public void getConstantTuning() {
        subsystems.get(outputCounter).getConstantTuning();
    }
}
