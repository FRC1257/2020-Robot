package frc.robot.commands.auto.segmented;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveDistanceCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SegTopAuto extends SequentialCommandGroup {

    public SegTopAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {

        addCommands(new SegDriveAndShoot(drivetrain, indexer, shooter, intake, 2.85),
            new DriveDistanceCommand(drivetrain, -5.652)
        );
    }
}
