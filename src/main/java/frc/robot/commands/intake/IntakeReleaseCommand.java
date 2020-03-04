package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeReleaseCommand extends InstantCommand {

    private final Intake intake;
    
    public IntakeReleaseCommand(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.toggleReleaseIntake();
    }
}
