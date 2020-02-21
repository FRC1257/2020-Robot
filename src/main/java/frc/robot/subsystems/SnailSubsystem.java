package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SnailSubsystem extends SubsystemBase {

    public abstract void outputValues();
    public abstract void setConstantTuning();
    public abstract void getConstantTuning();
}
