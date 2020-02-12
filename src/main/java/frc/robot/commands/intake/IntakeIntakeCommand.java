package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 * Intakes the power cells into the intake 
 */
public class IntakeIntakeCommand extends CommandBase {

    private final Intake intake;

    public IntakeIntakeCommand(Intake intake) {
        this.intake = intake;        
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.intake();
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