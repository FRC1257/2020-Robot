package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SnailSubsystem extends SubsystemBase {

    public abstract void update();
    public abstract void initLogging();
    public abstract void displayShuffleboard();
    public abstract void tuningInit();
    public abstract void tuningPeriodic();
}
