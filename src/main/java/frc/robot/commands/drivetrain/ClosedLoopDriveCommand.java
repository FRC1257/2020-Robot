package frc.robot.commands.drivetrain;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ClosedLoopDriveCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final XboxController controller;
    private double speedForward;
    private double speedTurn;

    public ClosedLoopDriveCommand(Drivetrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (controller.getAButton()) {
            speedForward = controller.getY(Hand.kLeft);
            speedTurn = controller.getX(Hand.kLeft);
        } else if (controller.getBumper(Hand.kRight)) {
            speedForward = controller.getY(Hand.kRight);
            speedTurn = controller.getX(Hand.kLeft);
        } else if (controller.getBumper(Hand.kLeft)) {
            speedForward = controller.getY(Hand.kLeft);
            speedTurn = controller.getX(Hand.kRight);
        } else {
            // intentionally empty
        }

        drivetrain.closedLoopDrive(-speedForward * DRIVE_MAX_VEL, 
            speedTurn * DRIVE_MAX_ROT);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
