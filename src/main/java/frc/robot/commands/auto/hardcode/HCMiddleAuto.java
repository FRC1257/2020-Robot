package frc.robot.commands.auto.hardcode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveDistanceCommand;
import frc.robot.commands.drivetrain.TurnAngleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class HCMiddleAuto extends SequentialCommandGroup {

    public HCMiddleAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, Intake intake) {
        
        addCommands(new TurnAngleCommand(drivetrain, 36.6),
            new DriveDistanceCommand(drivetrain, 2.85),
            new TurnAngleCommand(drivetrain, -36.6),
            new DriveDistanceCommand(drivetrain, 1.0),
            new PrepareAndShoot(drivetrain, indexer, shooter, intake),
            new DriveDistanceCommand(drivetrain, -5.652)
        );
    }
}