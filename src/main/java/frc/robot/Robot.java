package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;


public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;
    Indexer indexer;
    Intake intake;
    int outputCounter;
    @Override
    public void robotInit() {
        indexer = new Indexer();
        intake = new Intake();

        robotContainer = new RobotContainer();
        outputCounter = 0;
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        outputValues();
    }
    
    private void outputValues() {
        switch (outputCounter) {
            case 0:
                intake.outputValues();
                break;
            case 1:
                indexer.outputValues();
                break;
        }
        outputCounter = (outputCounter + 1) % 10;
    }
}