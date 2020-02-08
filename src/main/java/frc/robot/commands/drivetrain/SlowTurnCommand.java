package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class SlowTurnCommand extends InstantCommand {

    public SlowTurnCommand(Drivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        drivetrain.toggleTurnSlowdown();
    }
}
