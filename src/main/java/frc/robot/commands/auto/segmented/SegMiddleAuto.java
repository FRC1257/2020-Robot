package frc.robot.commands.auto.segmented;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveDistanceCommand;
import frc.robot.commands.drivetrain.TurnAngleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SegMiddleAuto extends SequentialCommandGroup {

    public SegMiddleAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {
        
        addCommands(new TurnAngleCommand(drivetrain, 36.6),
            new DriveDistanceCommand(drivetrain, 2.85),
            new TurnAngleCommand(drivetrain, -36.6),
            new PrepareAndShoot(drivetrain, indexer, shooter, intake, 1.0),
            new DriveDistanceCommand(drivetrain, -5.652)
        );
    }
}