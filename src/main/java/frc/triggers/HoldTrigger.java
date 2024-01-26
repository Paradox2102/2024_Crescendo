package frc.triggers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class HoldTrigger implements BooleanSupplier {
    private boolean m_state = false;

    public HoldTrigger (Trigger trigger) {
        trigger.whileTrue(new InstantCommand(() -> m_state = true));
        trigger.whileFalse(new InstantCommand(() -> m_state = false));
    }

    public boolean getAsBoolean() {
        return m_state;
    }
}