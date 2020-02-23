package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 * Ejects the power cells from the robot
 */
public class IntakeEjectCommand extends CommandBase {

    private final Intake intake;

    public IntakeEjectCommand(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.eject();
    }

    @Override
    public void end(boolean interrupted) {
        intake.neutral();
    }

    @Override
    public boolean isFinished() {
        return false;
    } 
}