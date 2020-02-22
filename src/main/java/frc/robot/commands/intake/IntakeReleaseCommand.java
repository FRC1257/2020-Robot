package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeReleaseCommand extends CommandBase {

    private final Intake intake;
    
    public IntakeReleaseCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.toggleReleaseIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.toggleReleaseIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
