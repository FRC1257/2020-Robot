package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.segmented.*;
import frc.robot.commands.auto.trajectory.*;
import frc.robot.commands.drivetrain.DriveDistanceCommand;
import frc.robot.commands.drivetrain.ManualDriveCommand;
import frc.robot.commands.drivetrain.ReverseDriveCommand;
import frc.robot.commands.drivetrain.SlowTurnCommand;
import frc.robot.commands.drivetrain.TurnAngleCommand;
import frc.robot.commands.elevator.ManualElevatorCommand;
import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.elevator.ToggleElevatorLockCommand;
import frc.robot.commands.indexer.IndexerLowerCommand;
import frc.robot.commands.indexer.IndexerNeutralCommand;
import frc.robot.commands.indexer.IndexerRaiseCommand;
import frc.robot.commands.indexer.IndexerShootCommand;
import frc.robot.commands.intake.IntakeEjectCommand;
import frc.robot.commands.intake.IntakeIntakeCommand;
import frc.robot.commands.intake.IntakeNeutralCommand;
import frc.robot.commands.intake.IntakeReleaseCommand;
import frc.robot.commands.shooter.ShooterNeutralCommand;
import frc.robot.commands.shooter.ShooterShootCommand;
import frc.robot.subsystems.*;
import frc.robot.util.Gyro;
import frc.robot.util.SnailController;

import java.util.ArrayList;

import static frc.robot.Constants.CONTROLLER_DRIVER_ID;
import static frc.robot.Constants.CONTROLLER_OPERATOR_ID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final SnailController driveController;
    private final SnailController operatorController;
    
    private final ArrayList<SnailSubsystem> subsystems;
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Indexer indexer;
    private final Elevator elevator;
    private final Shooter shooter;

    private SendableChooser<Constants.AutoType> autoTypeChooser;
    private SendableChooser<Constants.AutoPosition> autoPositionChooser;
    private SendableChooser<Constants.AutoGoal> autoGoalChooser;

    private int outputCounter;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveController = new SnailController(CONTROLLER_DRIVER_ID);
        operatorController = new SnailController(CONTROLLER_OPERATOR_ID);

        intake = new Intake();
        intake.setDefaultCommand(new IntakeNeutralCommand(intake));
        
        indexer = new Indexer();
        indexer.setDefaultCommand(new IndexerNeutralCommand(indexer));

        elevator = new Elevator();
        elevator.setDefaultCommand(new ManualElevatorCommand(elevator, operatorController::getLeftY));

        shooter = new Shooter();
        shooter.setDefaultCommand(new ShooterNeutralCommand(shooter));
        
        drivetrain = new Drivetrain();
        drivetrain.setDefaultCommand(new ManualDriveCommand(drivetrain, driveController::getDriveForward,
            driveController::getDriveTurn));
        
        subsystems = new ArrayList<>();
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
        driveController.getButton(Button.kY.value).whenPressed(new ReverseDriveCommand(drivetrain));
        driveController.getButton(Button.kStart.value).whenPressed(new SlowTurnCommand(drivetrain));
        driveController.getButton(Button.kB.value).whileActiveOnce(new TurnAngleCommand(drivetrain, 90));
        driveController.getButton(Button.kX.value).whileActiveOnce(new TurnAngleCommand(drivetrain, -90));

        // Intake Bindings
        operatorController.getButton(Button.kBack.value).whileHeld(new IntakeReleaseCommand(intake));
        operatorController.getButton(Button.kA.value).whileActiveOnce(new IntakeEjectCommand(intake));
        operatorController.getButton(Button.kB.value).whileActiveOnce(new IntakeIntakeCommand(intake));

        // Indexer Bindings
        operatorController.getButton(Button.kY.value).whileActiveOnce(new IndexerRaiseCommand(indexer));
        // operatorController.getButton(Button.kY.value).whenPressed(new IndexerPIDCommand(indexer));
        operatorController.getButton(Button.kX.value).whileActiveOnce(new IndexerLowerCommand(indexer));
        operatorController.getTrigger(Hand.kRight).whileActiveOnce(new IndexerShootCommand(indexer, shooter,
            () -> operatorController.getBumper(Hand.kRight)));

        // Elevator Bindings
        // operatorController.getButton(Button.kX.value).whileActiveOnce(new ElevatorPIDCommand(elevator));
        operatorController.getButton(Button.kStart.value).whenPressed(new ToggleElevatorLockCommand(elevator));

        // Shooting Bindings
        operatorController.getTrigger(Hand.kLeft).whileActiveOnce(new ShooterShootCommand(shooter));
        // operatorController.getTrigger(Hand.kLeft).whileActiveOnce(new ShooterPIDCommand(shooter));
    }

    public void configureAutoChoosers() {
        autoTypeChooser = new SendableChooser<>();
        autoPositionChooser = new SendableChooser<>();
        autoGoalChooser = new SendableChooser<>();

        autoTypeChooser.setDefaultOption("Segmented", Constants.AutoType.SEGMENTED);
        autoTypeChooser.addOption("Trajectory", Constants.AutoType.TRAJECTORY);

        autoPositionChooser.setDefaultOption("Top Start", Constants.AutoPosition.TOP);
        autoPositionChooser.addOption("Middle Start", Constants.AutoPosition.MIDDLE);
        autoPositionChooser.addOption("Bottom Start", Constants.AutoPosition.BOTTOM);

        autoGoalChooser.setDefaultOption("Default Drive", Constants.AutoGoal.DEFAULT);
        autoGoalChooser.addOption("Trench", Constants.AutoGoal.TRENCH);
        autoGoalChooser.addOption("Generator Top", Constants.AutoGoal.GEN_TOP);
        autoGoalChooser.addOption("Generator Bottom", Constants.AutoGoal.GEN_BOTTOM);

        SmartDashboard.putData(autoTypeChooser);
        SmartDashboard.putData(autoPositionChooser);
        SmartDashboard.putData(autoGoalChooser);
    }

    public Command getAutoCommand() {
        Constants.AutoType type = autoTypeChooser.getSelected();
        Constants.AutoPosition position = autoPositionChooser.getSelected();
        Constants.AutoGoal goal = autoGoalChooser.getSelected();

        if (type == Constants.AutoType.SEGMENTED) {
            switch (position) {
                case TOP:
                    return new SegTopAuto(drivetrain, indexer, shooter, intake);
                case MIDDLE:
                    return new SegMiddleAuto(drivetrain, indexer, shooter, intake);
                case BOTTOM:
                    return new SegBottomAuto(drivetrain, indexer, shooter, intake);
            }
        }

        if (type == Constants.AutoType.TRAJECTORY) {
            if (position == Constants.AutoPosition.BOTTOM) return new BottomAuto(drivetrain, indexer, shooter, intake);

            switch (goal) {
                case TRENCH:
                    if (position == Constants.AutoPosition.TOP) return new TopTrenchAuto(drivetrain, indexer, shooter, intake);
                    else if (position == Constants.AutoPosition.MIDDLE) return new MiddleTrenchAuto(drivetrain, indexer, shooter, intake);
                case GEN_TOP: 
                    if (position == Constants.AutoPosition.TOP) return new TopGenTopAuto(drivetrain, indexer, shooter, intake);
                    else if (position == Constants.AutoPosition.MIDDLE) return new MiddleGenTopAuto(drivetrain, indexer, shooter, intake);
                case GEN_BOTTOM:
                    if (position == Constants.AutoPosition.TOP) return new TopGenBottomAuto(drivetrain, indexer, shooter, intake);
                    else if (position == Constants.AutoPosition.MIDDLE) return new MiddleGenBottomAuto(drivetrain, indexer, shooter, intake);
                case DEFAULT: // will go to drive distance command
            }
        }

        return new DriveDistanceCommand(drivetrain, 2);
    }

    public void outputValues() {
        if (outputCounter < subsystems.size()) {
            subsystems.get(outputCounter).outputValues();
        }
        else {
            Gyro.getInstance().outputValues();
        }

        outputCounter = (outputCounter + 1) % (subsystems.size() + 1);
    }

    public void setConstantTuning() {
        subsystems.forEach(SnailSubsystem::setConstantTuning);
    }

    public void getConstantTuning() {
        if (outputCounter < subsystems.size()) {
            subsystems.get(outputCounter).getConstantTuning();
        }
    }
}
