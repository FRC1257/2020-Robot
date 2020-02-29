package frc.robot.commands.auto.hardcode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveDistanceCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class HCTopAuto extends SequentialCommandGroup {

    public HCTopAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {

        addCommands(new DriveDistanceCommand(drivetrain, 3.7),
            new PrepareAndShoot(drivetrain, indexer, shooter, intake),
            new DriveDistanceCommand(drivetrain, -5.652)
            );
    }
}


