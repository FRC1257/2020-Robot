package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxTrigger extends Trigger {
    
    private final XboxController controller;
    private final Hand hand;

    public XboxTrigger(XboxController controller, Hand hand) {
        this.controller = controller;
        this.hand = hand;
    }

    @Override
    public boolean get() {
        return controller.getTriggerAxis(hand) > 0.5;
    }
}
