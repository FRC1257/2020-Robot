package frc.robot.commands.tian;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FunPlusPhoenix;

public class ManualDriveCommand extends CommandBase {

    private final FunPlusPhoenix drivetrain;
    private final XboxController controller;

    public ManualDriveCommand(FunPlusPhoenix drivetrain, XboxController controller) {
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
        drivetrain.manualDrive(-controller.getY(Hand.kLeft), controller.getX(Hand.kRight));
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
