package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.trajectory.Trajectories;

public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;
    private Command autoCommand;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        Trajectories.setUpTrajectories();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.outputValues();
        if (SmartDashboard.getBoolean("Testing", false)) robotContainer.getConstantTuning();
    }

    @Override
    public void autonomousInit() {
        autoCommand = robotContainer.getAutoCommand();

        if (autoCommand != null) {
            autoCommand.schedule();
        }

        robotContainer.resetIndexer();
    }

    @Override
    public void teleopInit() {
        if (autoCommand != null) {
            autoCommand.cancel();
        }

        robotContainer.resetIndexer();
        if (SmartDashboard.getBoolean("Testing", false)) robotContainer.setConstantTuning();
    }
}