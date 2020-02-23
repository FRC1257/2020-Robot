package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class SlowTurnCommand extends InstantCommand {

    private final Drivetrain drivetrain;

    public SlowTurnCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.toggleTurnSlowdown();
    }
}
